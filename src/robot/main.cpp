#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>   // esp_reset_reason()
#include <esp_mac.h>      // esp_read_mac()
#include <esp_log.h>      // esp_log_level_set/get (mute PCNT pull-up noise)
#include <esp_task_wdt.h> // loop watchdog
#include <driver/pulse_cnt.h>  // PCNT quadrature decode
#include "protocol.h"
#include "config_robot.h"
#include "kinematics.h"     // mecanumMix() + forwardKinematics() + mixInverse() + slewQuad()
#include "governor.h"       // speedGovernorScale() — cross-wheel sync
#include "body_loop.h"      // bodyCorrection() — body-space outer loop
#include "control_math.h"   // crc8()
#include "safety.h"         // linkAgeMs(), linkGateCheck(), driveFaultStep(), driveGateStep()

// Direction-pin masks read by the task-watchdog ISR hook (src/robot/wdt_hook.cpp).
extern volatile uint32_t g_motorDirMaskLo;
extern volatile uint32_t g_motorDirMaskHi;

// ---------------- Motor pin map (see docs/pinout.md) ----------------
// Wheel layout (top-down view, robot facing forward):
//   M1 (FL) --- M2 (FR)
//   M3 (RL) --- M4 (RR)
struct Motor {
  uint8_t pwm;
  uint8_t inA;
  uint8_t inB;
  uint8_t encA;
  uint8_t encB;
};

// Slot i drives physical wheel i. Validated by solo-PWM observation.
// 2026-05-31 body swap: rear motors physically traded corners (the old slot 2
// hardware now sits at RR, old slot 3 hardware at RL) and both ran backward on
// +PWM. Wires too short to re-route, so fixed in software: rear array entries
// swapped (so each index drives its true corner) and inA/inB swapped on both
// rear entries to invert direction (+cmd -> forward). Front pair unchanged.
// Encoder pin pairs (encA/encB) travel with their motor, so encSign[] is
// unchanged. FR (slot 1) encoder was dead (loose power cable on the encoder
// supply) — fixed 2026-05-31, now reads full scale; back on closed loop.
static const Motor motors[4] = {
  { 18,  5, 19, 13, 17 },  // FL  motor=(18, 5,19)  encoder=(13,17)
  { 21, 23, 22, 16,  4 },  // FR  motor=(21,23,22)  encoder=(16, 4)
  { 25, 27, 33, 39, 36 },  // RL  was old-slot3 hw=(25,33,27); inA<->inB swapped to invert  encoder=(39,36)
  { 26, 32, 14, 35, 34 },  // RR  was old-slot2 hw=(26,14,32); inA<->inB swapped to invert  encoder=(35,34)
};

// PWM_FREQ / PWM_RES / PWM_MAX / DEADBAND now in config_robot.h.

// Derived from solo-PWM test: +PWM raw_tps signs [-, +, -, +] for [FL,FR,RL,RR].
static const int8_t encSign[4]   = { -1, +1, -1, +1 };

// Per-wheel open-loop override: pure feed-forward PWM, no P/I (for a dead
// encoder). All four encoders work now, so all closed-loop. Kept as a knob:
// flip an entry true if that encoder fails, so its wheel still drives.
static const bool openLoop[4] = { false, false, false, false };

// Full 4x quadrature decode in HARDWARE: one ESP32 pulse-counter (PCNT) unit per
// wheel, two channels each — A edges steered by B's level, B edges by A's. It
// replaces a GPIO interrupt on every edge of both channels (BUG-008 decode,
// 2026-06-02): ~34,000 interrupts/sec at full speed, each running two
// digitalRead()s on the control core. PCNT counts in silicon, so there is no
// per-edge CPU, no edge lost to interrupt latency, and its glitch filter
// (ENC_GLITCH_NS) drops motor-current spikes at the pin. A wheel dithering one
// channel still nets ~0, as with the old transition table.
// Sign convention is IDENTICAL to that table (state=(A<<1)|B, +1 along
// 00->10->11->01->00: e.g. A rising while B is low counts +1), and it is still 4
// counts per quadrature cycle — so encSign[] and the MAX_TPS[] scale stay valid.
// See [[wheel-sync-and-encsign]].
// The hardware counter is 16-bit. accum_count makes the driver extend it by adding
// ±ENC_PCNT_LIMIT in an overflow interrupt — one per ~4 s per wheel at full speed.
static const int ENC_PCNT_LIMIT = 32000;
static pcnt_unit_handle_t encUnit[4] = { nullptr, nullptr, nullptr, nullptr };
static bool encReady = false;   // true once all four units are counting

// Bench fault test (`d <slot>` in test mode): that wheel's encoder reads frozen,
// exactly like a disconnected encoder, so the drive-fault stop can be exercised on
// a healthy cart. Cleared by `d -1` and on leaving test mode. -1 = off.
static int8_t simDeadEnc   = -1;
static long   simDeadCount = 0;

static bool isInputOnly(uint8_t pin) {
  return pin == 34 || pin == 35 || pin == 36 || pin == 39;
}

static bool setupEncoder(uint8_t i) {
  const Motor& m = motors[i];
  pinMode(m.encA, isInputOnly(m.encA) ? INPUT : INPUT_PULLUP);
  pinMode(m.encB, isInputOnly(m.encB) ? INPUT : INPUT_PULLUP);

  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -ENC_PCNT_LIMIT;
  ucfg.high_limit =  ENC_PCNT_LIMIT;
  ucfg.flags.accum_count = 1;
  pcnt_glitch_filter_config_t fcfg = {};
  fcfg.max_glitch_ns = ENC_GLITCH_NS;
  pcnt_chan_config_t acfg = {};          // counts A edges, direction from B's level
  acfg.edge_gpio_num  = m.encA;
  acfg.level_gpio_num = m.encB;
  pcnt_chan_config_t bcfg = {};          // counts B edges, direction from A's level
  bcfg.edge_gpio_num  = m.encB;
  bcfg.level_gpio_num = m.encA;

  pcnt_unit_handle_t    u   = nullptr;
  pcnt_channel_handle_t chA = nullptr, chB = nullptr;
  esp_err_t e = pcnt_new_unit(&ucfg, &u);
  if (e == ESP_OK) e = pcnt_unit_set_glitch_filter(u, &fcfg);
  if (e == ESP_OK) e = pcnt_new_channel(u, &acfg, &chA);
  if (e == ESP_OK) e = pcnt_new_channel(u, &bcfg, &chB);
  // A: rising = -1 while B high, inverted (+1) while B low. B: rising = +1 while A
  // high, inverted (-1) while A low. Falling edges are the mirror image.
  if (e == ESP_OK) e = pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  if (e == ESP_OK) e = pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  if (e == ESP_OK) e = pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  if (e == ESP_OK) e = pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  // The limit watch points are what arm the overflow interrupt accum_count needs.
  if (e == ESP_OK) e = pcnt_unit_add_watch_point(u,  ENC_PCNT_LIMIT);
  if (e == ESP_OK) e = pcnt_unit_add_watch_point(u, -ENC_PCNT_LIMIT);
  if (e == ESP_OK) e = pcnt_unit_enable(u);
  if (e == ESP_OK) e = pcnt_unit_clear_count(u);
  if (e == ESP_OK) e = pcnt_unit_start(u);
  if (e != ESP_OK) {
    Serial.printf("encoder %u PCNT init FAILED: %s\n", i, esp_err_to_name(e));
    return false;
  }
  encUnit[i] = u;
  return true;
}

static void motorWrite(uint8_t idx, int16_t speed) {
  if (idx >= 4) return;
  const Motor& m = motors[idx];
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  if (speed > 0 && speed < DEADBAND) speed = DEADBAND;
  else if (speed < 0 && speed > -DEADBAND) speed = -DEADBAND;

  if (speed > 0) {
    digitalWrite(m.inA, HIGH);
    digitalWrite(m.inB, LOW);
    ledcWrite(m.pwm, speed);
  } else if (speed < 0) {
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, HIGH);
    ledcWrite(m.pwm, -speed);
  } else {
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, LOW);
    ledcWrite(m.pwm, 0);
  }
}

static void motorStopAll() {
  for (uint8_t i = 0; i < 4; i++) motorWrite(i, 0);
}

// ---------------- Battery / reset telemetry (#5) ----------------
// Returns pack voltage in volts, or -1 when no sensing is wired (BATT_ADC_PIN<0)
// so callers can omit the field rather than print a fabricated value.
static float readBattVolts() {
  if (BATT_ADC_PIN < 0) return -1.0f;
  float vadc = analogReadMilliVolts(BATT_ADC_PIN) / 1000.0f;
  return vadc * BATT_DIVIDER;
}

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";  // supply rail collapsed
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
}

// ---------------- Mecanum kinematics ----------------
// mecanumMix() now in include/kinematics.h (host-testable, no Arduino deps).

// ---------------- Per-wheel PI velocity control ----------------
// MAX_TPS / Kff / Kp / Ki / I_MAX now in config_robot.h.

struct PidState {
  float  integral;
  long   lastCount;
};
static PidState pid[4] = {};

// Command slew limiter: ramp each wheel cmd toward its mix target instead of
// stepping instantly. Four motors slamming 0->full at once draw near-stall
// current simultaneously (no back-EMF yet) and collapse a weak supply rail —
// worst during spin-in-place. Ramping staggers the current rise. (CMD_SLEW in config_robot.h.)
static int32_t curCmd[4] = { 0, 0, 0, 0 };

// Cross-wheel governor: low-passed group scale applied to all wheel commands.
static float govScale = 1.0f;

// Runtime feature toggles (RAM-only, no NVS). Initialized from the compile-time
// defaults so boot behaviour is unchanged; flipped live by the controller's mode
// bits (field) or the serial g/c/b commands (bench). A power cycle restores these
// defaults. Independent — any combination is valid. See config_robot.h.
static bool enableGovernor  = SYNC_GOVERNOR;
static bool enableClosedLoop = CLOSED_LOOP_DEFAULT;
static bool enableBodyLoop  = BODY_LOOP;

// Body-space outer loop: persistent state + IIR-filtered chassis-twist estimate.
static BodyLoopState bodyState = { 0, 0, 0, 0, 0, 0 };
static float bodyVxF = 0, bodyVyF = 0, bodyWF = 0, bodySF = 0;  // filtered vx,vy,omega,slip
static float lastBodyCorr[3] = { 0, 0, 0 };                     // last (dvx,dvy,dw) for telemetry

// Last PID step values per wheel (telemetry, governor and body-loop inputs)
static float lastTargetTps[4] = { 0, 0, 0, 0 };
static float lastMeasTps[4]   = { 0, 0, 0, 0 };
static float lastOutPwm[4]    = { 0, 0, 0, 0 };

// Per-wheel stall detection (telemetry): pinned near max PWM but barely moving.
static float stallMs[4]      = { 0, 0, 0, 0 };
static bool  wheelStalled[4] = { false, false, false, false };

// Drive fault (safety.h): a wheel driven hard with no encoder motion is switched
// off (latched) while the others keep driving. faultMask bit i = slot i is off.
static DriveFault faultState[4] = {};
static DriveGate  gate          = { false, true, 0 };  // link latched until the first neutral frame
static uint8_t    faultMask     = 0;
// Slot names as the RIDER sees them (protocol.h SLOT_RIDER_NAME): slots are named
// in the firmware frame, but the rider faces the firmware's rear.
#define RIDER_NAME SLOT_RIDER_NAME

static int deadWheelCount() {
  int n = 0;
  for (int i = 0; i < 4; i++) if (faultMask & (1u << i)) n++;
  return n;
}

// Automatic re-arm of a switched-off wheel while driving (safety.h WheelRecovery).
static WheelRecovery recov[4] = {};
static const WheelRecoveryCfg RECOV_CFG = { FAULT_RETRY_FIRST_MS, FAULT_RETRY_MAX_MS,
                                            FAULT_RETRY_RESET_MS, FAULT_TPS, FAULT_ROLL_MS };

// Wheel commands actually handed to pidStep last tick (after slew, governor and
// body correction). Telemetry `cmd=` reports these, not the raw packet mix.
static int32_t lastDriveCmd[4] = { 0, 0, 0, 0 };

// Motors held stopped (watchdog / e-stop / fault / test abort). The first drive
// tick after a stop re-baselines the encoders and resumes governed.
static bool wasStopped = true;

// Accumulated (32-bit) encoder count. Reads 0 for a unit that failed to init;
// encReady keeps the drive stopped in that case.
static long readCount(uint8_t i) {
  if ((int8_t)i == simDeadEnc) return simDeadCount;   // bench: simulated dead encoder
  int c = 0;
  if (encUnit[i]) pcnt_unit_get_count(encUnit[i], &c);
  return c;
}

// Body-space outer loop: drop integrals, filtered estimate, and correction so the
// loop starts from a clean chassis state (no stale heading bias). Called on every
// stop AND on every tick the body loop is disabled: otherwise a NO BODY -> FULL
// mode switch mid-drive applied the last rate-limited correction (up to +-200) and
// stale integrals in one step.
static void bodyLoopReset() {
  bodyState.ix = bodyState.iy = bodyState.iw = 0.0f;
  bodyState.dvx = bodyState.dvy = bodyState.dw = 0.0f;
  bodyVxF = bodyVyF = bodyWF = bodySF = 0.0f;
  lastBodyCorr[0] = lastBodyCorr[1] = lastBodyCorr[2] = 0.0f;
}

static void pidReset() {
  for (int i = 0; i < 4; i++) {
    pid[i].integral  = 0.0f;
    pid[i].lastCount = readCount(i);
    curCmd[i]        = 0;     // drop the slew ramp so resume starts from rest
    lastOutPwm[i]    = 0.0f;  // output slew restarts from zero too
    lastDriveCmd[i]  = 0;
    stallMs[i]       = 0.0f;
    wheelStalled[i]  = false;
    faultState[i].ms = 0.0f;
  }
  govScale = 1.0f;            // resume ungoverned, ramp in again from measurement
  bodyLoopReset();
}

// The actuator dynamics every pidStep branch shares: ±PWM_MAX clamp + PWM_SLEW from
// the last delivered output. Thin wrapper binds the tuning constants (clampSlew is
// the host-tested logic in control_math.h).
static inline float limitPwm(float desired, float prevOut, float dt) {
  return clampSlew(desired, prevOut, (float)PWM_MAX, PWM_SLEW * dt);
}

// Feed one wheel's delivered PWM + measurement to its drive-fault detector.
// Returns this wheel's bit if it tripped this tick.
static inline uint8_t faultCheck(int i, float out, float measuredTps, float dt) {
  bool trip = driveFaultStep(&faultState[i], out, measuredTps,
                             FAULT_PWM_FRAC * (float)PWM_MAX, FAULT_TPS,
                             FAULT_MS, dt * 1000.0f);
  return trip ? (uint8_t)(1u << i) : 0;
}

// cmd[i] in [-1000..+1000]. dt = the CONTROL step in seconds (clamped by the
// caller; drives slew, integral and fault timing). dtMeas = the TRUE time the
// encoder delta spans: measuring speed with the clamped dt read a 100 ms stall as
// double speed and dipped PWM for a tick. Returns a mask of wheels whose drive
// fault tripped this tick (0 = healthy); the caller switches those wheels off.
// Wheels already switched off (faultMask) are held at zero output here.
static uint8_t pidStep(const int32_t cmd[4], float dt, float dtMeas) {
  uint8_t tripped = 0;
  for (int i = 0; i < 4; i++) {
    // Signed encoder delta (apply sign to fix wiring inversions).
    long now   = readCount(i);
    long delta = now - pid[i].lastCount;
    pid[i].lastCount = now;

    // Glitch rejection: a real wheel can't exceed ~MAX_TPS, so a larger
    // per-tick delta is electrical noise from motor current transients. Clamp
    // it so the loop tracks real motion, not the spike. Without this, inrush
    // noise spikes the velocity estimate, the PID slams PWM to react, and the
    // current swing makes more noise — a self-sustaining limit cycle (worst on
    // vx+, all 4 motors inrushing forward together). The PCNT glitch filter now
    // drops most of that noise at the pin; this clamp stays as the backstop.
    long maxDelta = (long)(MAX_TPS[i] * 1.5f * dtMeas) + 2;
    if (delta >  maxDelta) delta =  maxDelta;
    if (delta < -maxDelta) delta = -maxDelta;
    float measuredTps = encSign[i] * (float)delta / dtMeas;

    // Switched off by a drive fault: no power (coast/brake per the driver's L/L
    // state), no integral, no fault timing. The measurement is still logged.
    if (faultMask & (1u << i)) {
      motorWrite(i, 0);
      pid[i].integral  = 0.0f;
      faultState[i].ms = 0.0f;
      lastTargetTps[i] = 0;
      lastMeasTps[i]   = measuredTps;
      lastOutPwm[i]    = 0.0f;
      continue;
    }

    // Feed-forward is per-wheel (a weak-battery wheel needs more PWM per tick/sec),
    // but the TARGET is a UNIFORM absolute speed referenced to the weakest wheel
    // (refTps = cmdRefTps) so cmd 1000 means the same ground speed on every wheel
    // and the cart drives straight. A strong wheel simply uses less of its range.
    const float refTps    = cmdRefTps();
    const float kff       = (float)PWM_MAX / MAX_TPS[i];  // per-wheel feed-forward
    float targetTps = ((float)cmd[i] / 1000.0f) * refTps;

    if (cmd[i] == 0) {
      // Commanded idle: ramp PWM down to 0 respecting PWM_SLEW instead of slamming
      // (a wheel pinned at PWM_MAX dropping to 0 in one tick defeats the anti-slam
      // intent and shocks the supply). The estop/watchdog path calls motorStopAll()
      // directly for an immediate hard stop, so safety stops are unaffected.
      // (BUG-011) DECAY the integral instead of hard-zeroing it. On a forward+turn
      // where one side's wheel target transiently crosses zero (fwd≈turn), a hard
      // dump desynced that wheel from its still-driving partner — an asymmetric
      // thrust the governor/body loop then read as yaw. A fast decay still clears on
      // a real stop (sub-tick over ~50ms) but does not slam a momentary zero-cross.
      pid[i].integral *= 0.5f;
      float out = limitPwm(0.0f, lastOutPwm[i], dt);
      motorWrite(i, (int16_t)lroundf(out));
      stallMs[i]       = 0.0f;
      wheelStalled[i]  = false;
      faultState[i].ms = 0.0f;
      lastTargetTps[i] = 0;
      lastMeasTps[i]   = measuredTps;
      lastOutPwm[i]    = out;
      continue;
    }

    // Dead-encoder wheels: open-loop feed-forward only. No error term (measured
    // is meaningless), so PWM tracks the commanded speed directly.
    if (openLoop[i]) {
      float out = kff * targetTps;
      out = constrain(out, -(float)PWM_MAX, (float)PWM_MAX);
      motorWrite(i, (int16_t)lroundf(out));
      pid[i].integral  = 0.0f;
      pid[i].lastCount = now;
      faultState[i].ms = 0.0f;            // no feedback to judge a fault by
      lastTargetTps[i] = targetTps;
      lastMeasTps[i]   = 0.0f;
      lastOutPwm[i]    = out;
      continue;
    }

    // Global open-loop TEST toggle (encoder healthy): feed-forward only, no P/I.
    // Distinct from the per-wheel openLoop[] dead-encoder branch above — here the
    // encoder works, so lastMeasTps stays REAL (the dead-encoder branch zeros it),
    // keeping the governor, body loop, telemetry and stall detection truthful
    // during an A/B. Apply the SAME actuator dynamics as the PI path (PWM_MAX
    // clamp + PWM_SLEW) so only the control law differs, not the actuator.
    if (!enableClosedLoop) {
      float out = limitPwm(kff * targetTps, lastOutPwm[i], dt);
      pid[i].integral  = 0.0f;            // clean re-enable (no stale windup)
      motorWrite(i, (int16_t)lroundf(out));
      tripped |= faultCheck(i, out, measuredTps, dt);  // a jam stalls open loop too
      lastTargetTps[i] = targetTps;
      lastMeasTps[i]   = measuredTps;     // REAL measurement (encoder healthy)
      lastOutPwm[i]    = out;
      continue;
    }

    float err = targetTps - measuredTps;

    // Compute the desired output from the CURRENT integral, then apply BOTH
    // actuator limits — the hard PWM_MAX clamp and the per-tick slew limiter —
    // to get the value actually delivered this tick.
    float desired = kff * targetTps + Kp * err + pid[i].integral;
    float out     = limitPwm(desired, lastOutPwm[i], dt);

    // Conditional-integration anti-windup: integrate only when the actuator is
    // NOT limited in the direction the error would push it. This now covers the
    // slew limiter too, not just the ±PWM_MAX clamp — the old test missed the
    // case where the integral kept winding while the output was slew-pinned far
    // below PWM_MAX, then overshot once the ramp released.
    bool limited = (out < desired - 0.001f && err > 0) ||
                   (out > desired + 0.001f && err < 0);
    if (!limited) {
      pid[i].integral += err * dt * Ki;
      if (pid[i].integral >  I_MAX) pid[i].integral =  I_MAX;
      if (pid[i].integral < -I_MAX) pid[i].integral = -I_MAX;
    }

    // Stall flag (telemetry): pinned near max PWM yet barely moving = held/jammed
    // or the supply rail collapsed. Surfaces WHICH wheel for the operator; the
    // governor already trims command to keep stall current down.
    float aout  = out < 0 ? -out : out;
    float ameas = measuredTps < 0 ? -measuredTps : measuredTps;
    if (aout >= STALL_PWM_FRAC * PWM_MAX && ameas < STALL_TPS_FRAC * MAX_TPS[i]) {
      stallMs[i] += dt * 1000.0f;
    } else {
      stallMs[i] = 0.0f;
    }
    wheelStalled[i] = stallMs[i] >= STALL_MS;

    motorWrite(i, (int16_t)lroundf(out));
    tripped |= faultCheck(i, out, measuredTps, dt);
    lastTargetTps[i] = targetTps;
    lastMeasTps[i]   = measuredTps;
    lastOutPwm[i]    = out;
  }
  return tripped;
}

// ---------------- Test mode (serial command driven) ----------------
// testMode + testSrc selects what feeds the drive step:
//   TS_MIX:    `t` command -> mecanumMix -> pidStep -> motorWrite (production path).
//   TS_DIRECT: `m` command -> motorWrite (bypass kinematics + PID, raw PWM per slot).
// Commands:
//   t <vx> <vy> <omega>     mix path (each -1000..+1000)
//   m <slot> <pwm>          direct path (slot 0..3, pwm -1023..+1023)
//   s                       stop (zero everything); enters test mode
//   r                       stop + zero encoder counters + PID + clear a drive fault
//                           (test mode only: in the field it was a mid-drive jolt)
//   x                       exit test mode (ESP-NOW control resumes), clear a fault;
//                           a no-op outside test mode
//   k                       keepalive (silent): any line feeds the test-link watchdog
//   d <slot>                bench fault test: freeze that wheel's encoder (d -1 = off)
//   g|c|b <0|1>             governor / closed loop / body loop on|off (test mode;
//                           in the field the controller's mode bits own these)
//   ?                       print one-shot status
// Failsafes: a test that drives stops after TEST_LINK_MS without a serial line
// (bench tools send `k` every 250 ms), and a radio e-stop aborts test mode.
enum TestSrc : uint8_t { TS_MIX, TS_DIRECT };
// volatile: read by onRecv on core 0 (inside the packet lock) while loop() on
// core 1 sets it.
static volatile bool testMode = false;
static TestSrc   testSrc  = TS_MIX;
static int16_t   slotPwm[4] = { 0, 0, 0, 0 };
static char      cmdBuf[64];
static uint8_t   cmdLen = 0;
static uint32_t  lastSerialLineMs = 0;   // millis() of the last complete serial line
static volatile bool radioEstopReq = false;  // set by onRecv during test mode

static long      prevEnc[4]  = { 0, 0, 0, 0 };
static uint32_t  prevTlmMs   = 0;
static uint32_t  lastTlmMs   = 0;

static int16_t clampCmd(int v) { return (int16_t)constrain(v, -1000, 1000); }
static int16_t clampPwm(int v) { return (int16_t)constrain(v, -PWM_MAX, PWM_MAX); }

// Forward decls — defined below near ESP-NOW state.
static void setPacketFromTest(int16_t vx, int16_t vy, int16_t omega);
static void getPacketSnapshot(CtrlPacket& out);

// Bench reset (`r`/`x`): every wheel back on, recovery backoff forgotten.
static void clearDriveFault(const char* why) {
  bool was = gate.fault;
  gate.fault = false;
  faultMask  = 0;
  for (int i = 0; i < 4; i++) { faultState[i].ms = 0.0f; recov[i] = WheelRecovery{}; }
  if (was) Serial.printf("FAULT cleared (%s)\n", why);
}

// Give one switched-off wheel its power back (recovery or neutral sticks).
static void rearmWheel(int i, uint32_t now, const char* why) {
  if (!(faultMask & (1u << i))) return;
  faultMask &= (uint8_t)~(1u << i);
  wheelRecoveryRearm(&recov[i], now);
  faultState[i].ms = 0.0f;
  pid[i].integral  = 0.0f;
  pid[i].lastCount = readCount(i);   // clean first delta
  if (faultMask == 0) gate.fault = false;
  Serial.printf("wheel %s (slot %d) re-armed (%s)\n", RIDER_NAME[i], i, why);
}

// Re-baseline every wheel's last count (PID + telemetry) to what readCount()
// returns now. Needed whenever readCount() can jump: `d` freezing/releasing an
// encoder used to produce a one-tick phantom speed spike (clamped to 1.5x MAX).
static void rebaselineCounts() {
  for (int i = 0; i < 4; i++) {
    pid[i].lastCount = readCount(i);
    prevEnc[i]       = pid[i].lastCount;
  }
}

// Leave test mode stopped: ESP-NOW control resumes from a zero packet.
static void exitTestMode() {
  testMode   = false;
  testSrc    = TS_MIX;
  simDeadEnc = -1;      // a simulated dead encoder never survives into field control
  for (int i = 0; i < 4; i++) slotPwm[i] = 0;
  setPacketFromTest(0, 0, 0);
  motorStopAll();
  pidReset();
  wasStopped = true;
}

static bool testDriving() {
  CtrlPacket p;
  getPacketSnapshot(p);
  if (p.vx != 0 || p.vy != 0 || p.omega != 0) return true;
  for (int i = 0; i < 4; i++) if (slotPwm[i] != 0) return true;
  return false;
}

static void handleCommand(char* line) {
  char* tok = strtok(line, " \t");
  if (!tok) return;
  char c = (char)tolower((unsigned char)tok[0]);
  switch (c) {
    case 't': {
      char* a = strtok(NULL, " \t");
      char* b = strtok(NULL, " \t");
      char* d = strtok(NULL, " \t");
      if (!a || !b || !d) { Serial.println("ERR usage: t <vx> <vy> <omega>"); return; }
      int16_t vx = clampCmd(atoi(a));
      int16_t vy = clampCmd(atoi(b));
      int16_t w  = clampCmd(atoi(d));
      testMode = true;
      testSrc  = TS_MIX;
      setPacketFromTest(vx, vy, w);
      Serial.printf("OK t %d %d %d\n", vx, vy, w);
      break;
    }
    case 'm': {
      char* a = strtok(NULL, " \t");
      char* b = strtok(NULL, " \t");
      if (!a || !b) { Serial.println("ERR usage: m <slot> <pwm>"); return; }
      int slot = atoi(a);
      int pwm  = atoi(b);
      if (slot < 0 || slot > 3) { Serial.println("ERR slot 0..3"); return; }
      testMode = true;
      testSrc  = TS_DIRECT;
      for (int i = 0; i < 4; i++) if (i != slot) slotPwm[i] = 0;
      slotPwm[slot] = clampPwm(pwm);
      Serial.printf("OK m %d %d\n", slot, slotPwm[slot]);
      break;
    }
    case 's':
      testMode = true;
      for (int i = 0; i < 4; i++) slotPwm[i] = 0;
      setPacketFromTest(0, 0, 0);
      Serial.println("OK s");
      break;
    case 'r':
      // Test mode only. In the field it zeroed the output slew mid-drive (PWM fell
      // from ~800 to 25 in one tick, then ramped back) and cleared a fault with the
      // sticks still pushed. It also stops: clearing a fault must never resume a
      // latched `t`/`m` command.
      if (!testMode) { Serial.println("ERR r needs test mode (send s first)"); return; }
      for (int i = 0; i < 4; i++) slotPwm[i] = 0;
      setPacketFromTest(0, 0, 0);
      motorStopAll();
      for (int i = 0; i < 4; i++) if (encUnit[i]) pcnt_unit_clear_count(encUnit[i]);
      if (simDeadEnc >= 0) simDeadCount = 0;   // the frozen wheel "zeroes" too
      pidReset();
      wasStopped = true;
      // Baseline from the post-zero counts (not a hard 0) so the first delta after
      // `r` doesn't show a phantom velocity from counts that landed meanwhile.
      rebaselineCounts();
      clearDriveFault("r");
      Serial.println("OK r");
      break;
    case 'x':
      if (!testMode) { Serial.println("OK x (not in test mode)"); break; }
      exitTestMode();
      clearDriveFault("x");
      Serial.println("OK x");
      break;
    case 'k':
      break;  // keepalive: pollSerial already stamped lastSerialLineMs
    case 'd': {
      // Test mode only: the field path never sees a simulated fault.
      char* a = strtok(NULL, " \t");
      int slot = a ? atoi(a) : -1;
      if (!testMode)                { Serial.println("ERR d needs test mode (send s first)"); return; }
      if (slot < -1 || slot > 3)    { Serial.println("ERR usage: d <slot 0..3 | -1>"); return; }
      if (slot >= 0) {
        int c = 0;
        if (encUnit[slot]) pcnt_unit_get_count(encUnit[slot], &c);
        simDeadCount = c;
      }
      simDeadEnc = (int8_t)slot;
      rebaselineCounts();
      if (slot >= 0) Serial.printf("OK d %d (%s encoder frozen: bench fault test)\n", slot, RIDER_NAME[slot]);
      else           Serial.println("OK d -1");
      break;
    }
    case 'g': {
      char* a = strtok(NULL, " \t");
      if (a) enableGovernor = atoi(a) != 0;
      Serial.printf("OK g %d\n", enableGovernor);
      break;
    }
    case 'c': {
      char* a = strtok(NULL, " \t");
      if (a) enableClosedLoop = atoi(a) != 0;
      Serial.printf("OK c %d\n", enableClosedLoop);
      break;
    }
    case 'b': {
      char* a = strtok(NULL, " \t");
      if (a) enableBodyLoop = atoi(a) != 0;
      Serial.printf("OK b %d\n", enableBodyLoop);
      break;
    }
    case '?': {
      CtrlPacket p;
      getPacketSnapshot(p);
      Serial.printf("STATUS testMode=%d src=%s en=[gov=%d cl=%d body=%d] packet vx=%d vy=%d omega=%d slotPwm=[%d %d %d %d] fault=%u simDead=%d\n",
                    testMode ? 1 : 0, testSrc == TS_DIRECT ? "DIRECT" : "MIX",
                    enableGovernor, enableClosedLoop, enableBodyLoop,
                    p.vx, p.vy, p.omega,
                    slotPwm[0], slotPwm[1], slotPwm[2], slotPwm[3], faultMask, (int)simDeadEnc);
      break;
    }
    default:
      Serial.printf("ERR unknown '%c'\n", c);
  }
}

static void pollSerial() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      lastSerialLineMs = millis();
      if (cmdLen > 0) handleCommand(cmdBuf);
      cmdLen = 0;
      continue;
    }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = (char)c;
  }
}

// Control-state fields shared by the TLM stream and the 2 Hz status line, built
// once so the two can't drift apart. tools/tlm.py reads them by field name.
// cmd= is what pidStep was actually handed (after slew, governor and body
// correction), so it lines up with pwm= and raw_tps= on the same line. fault= is
// the drive-fault wheel mask (bit 0 = FL).
static void fmtCtrlState(char* buf, size_t n) {
  snprintf(buf, n,
           "cmd=[%ld %ld %ld %ld] gov=%.2f en=[%d %d %d] body=[%.0f %.0f %.0f] s=%.0f corr=[%.0f %.0f %.0f] pwm=[%.0f %.0f %.0f %.0f] fault=%u",
           (long)lastDriveCmd[0], (long)lastDriveCmd[1], (long)lastDriveCmd[2], (long)lastDriveCmd[3],
           govScale,
           enableGovernor, enableClosedLoop, enableBodyLoop,
           bodyVxF, bodyVyF, bodyWF, bodySF,
           lastBodyCorr[0], lastBodyCorr[1], lastBodyCorr[2],
           lastOutPwm[0], lastOutPwm[1], lastOutPwm[2], lastOutPwm[3],
           faultMask);
}

// " vbat=12.34", or empty when no sensing is wired.
static void fmtBatt(char* buf, size_t n) {
  buf[0] = '\0';
  float vb = readBattVolts();
  if (vb >= 0) snprintf(buf, n, " vbat=%.2f", vb);
}

static void emitTlm(uint32_t now) {
  // Skip a same-millisecond re-entry: dividing by a 1 ms floor inflates raw_tps
  // ~50x and shows a phantom spike on the readout used to diagnose stalls.
  if (prevTlmMs != 0 && now == prevTlmMs) return;
  uint32_t dt_ms = (prevTlmMs == 0) ? 50 : (now - prevTlmMs);
  float dt = dt_ms / 1000.0f;
  long cnt[4], delta[4];
  for (int i = 0; i < 4; i++) {
    cnt[i]    = readCount(i);
    delta[i]  = cnt[i] - prevEnc[i];
    prevEnc[i] = cnt[i];
  }
  prevTlmMs = now;

  char state[200], batt[24];
  fmtCtrlState(state, sizeof(state));
  fmtBatt(batt, sizeof(batt));

  Serial.printf("TLM ms=%lu %s raw_tps=[%.1f %.1f %.1f %.1f] cnt=[%ld %ld %ld %ld]%s\n",
                (unsigned long)now, state,
                (float)delta[0]/dt, (float)delta[1]/dt, (float)delta[2]/dt, (float)delta[3]/dt,
                cnt[0], cnt[1], cnt[2], cnt[3], batt);
}

// ---------------- ESP-NOW receive ----------------
static volatile uint32_t lastPacketMs = 0;
static CtrlPacket lastPacket = { 0, 0, 0, 0, 0, 0, 0 };
static portMUX_TYPE pktMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t crcDrops = 0;      // frames rejected on bad CRC
static volatile uint32_t foreignDrops = 0;  // frames from a second transmitter
static bool espNowReady = false;        // false until radio init + recv cb succeed
static LinkGate linkGate = {};          // touched only from onRecv (WiFi task)
static uint8_t  senderMac[6] = {};      // last accepted transmitter (under pktMux)
static bool     senderKnown  = false;   // (under pktMux)

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(CtrlPacket)) return;

  CtrlPacket in;
  memcpy(&in, data, sizeof(in));

  // Integrity (#4): drop frames whose crc8 over the leading bytes mismatches.
  if (in.crc != crc8((const uint8_t*)&in, offsetof(CtrlPacket, crc))) {
    crcDrops = crcDrops + 1;   // sole writer (WiFi task); `++` on volatile is deprecated
    return;
  }

#if ESPNOW_ENCRYPT
  // Registering an encrypted peer does not stop OTHER devices' plaintext frames
  // from reaching this callback, and after WATCHDOG_MS of silence the link gate
  // would lock onto one. With encryption on, only the configured controller counts.
  if (memcmp(info->src_addr, CONTROLLER_MAC, 6) != 0) {
    foreignDrops = foreignDrops + 1;
    return;
  }
#endif

  // Serial test mode owns the drive and ignores radio commands, but a radio
  // E-STOP still stops it: on the floor, the handheld e-stop is the only stop
  // within reach. From any sender, stale or not — an e-stop is never stale.
  if (testMode) {
    if (in.flags & CTRL_FLAG_ESTOP) radioEstopReq = true;
    return;
  }

  // One locked sender, newer seq only; reopens after WATCHDOG_MS of silence so a
  // rebooted controller is picked up at once (safety.h linkGateCheck).
  LinkVerdict v = linkGateCheck(&linkGate, info->src_addr, in.seq, millis(), WATCHDOG_MS);
  if (v == LINK_FOREIGN) foreignDrops = foreignDrops + 1;
  if (v != LINK_ACCEPT) return;

  // Re-check test mode INSIDE the lock: a serial `t`/`s` landing between the check
  // above and this write would otherwise be overwritten by this radio frame, and
  // test mode then keeps that frame fresh. handleCommand sets testMode before its
  // own locked write, so whichever order the two cores take, the test packet wins.
  portENTER_CRITICAL(&pktMux);
  if (!testMode) {
    lastPacket   = in;
    lastPacketMs = millis();
  }
  memcpy(senderMac, info->src_addr, 6);   // status back-channel destination
  senderKnown = true;
  portEXIT_CRITICAL(&pktMux);
}

static void setPacketFromTest(int16_t vx, int16_t vy, int16_t omega) {
  portENTER_CRITICAL(&pktMux);
  lastPacket.vx    = vx;
  lastPacket.vy    = vy;
  lastPacket.omega = omega;
  lastPacket.flags = 0;
  lastPacket.seq++;
  lastPacketMs     = millis();
  portEXIT_CRITICAL(&pktMux);
}

static void getPacketSnapshot(CtrlPacket& out) {
  portENTER_CRITICAL(&pktMux);
  out = lastPacket;
  portEXIT_CRITICAL(&pktMux);
}

static void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Factory STA MAC from eFuse. WiFi.macAddress() printed 00:00:00:00:00:00 here on
  // Arduino core 3.x (the netif isn't up yet), and secrets.h ROBOT_MAC is copied
  // from this line.
  uint8_t mac[6] = {};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Robot MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }

#if ESPNOW_ENCRYPT
  // Set the primary key, then register the controller as an encrypted peer so its
  // frames decrypt (#3). Frames from other senders are dropped in onRecv by MAC.
  esp_now_set_pmk((const uint8_t*)ESPNOW_PMK);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = true;
  memcpy(peer.lmk, ESPNOW_LMK, 16);
  if (esp_now_add_peer(&peer) != ESP_OK) Serial.println("add controller peer FAILED");
  Serial.println("ESP-NOW encryption ENABLED");
#endif

  esp_now_register_recv_cb(onRecv);
  espNowReady = true;
  Serial.println("ESP-NOW listening");
}

// ---------------- Status back-channel (robot -> controller) ----------------
// Every STATUS_PERIOD_MS, tell the transmitter the robot last accepted what it is
// doing (protocol.h StatusPacket): which wheel a drive fault switched off, link
// latch, test mode, encoder failure. The controller shows it instead of a bare
// ONLINE. Best effort: a lost status frame is replaced 200 ms later.
static void sendStatus(uint32_t now) {
  static uint32_t lastMs = 0;
  static uint8_t  seq    = 0;
  if (!espNowReady || now - lastMs < STATUS_PERIOD_MS) return;
  lastMs = now;

  uint8_t mac[6];
  bool known;
  portENTER_CRITICAL(&pktMux);
  known = senderKnown;
  memcpy(mac, senderMac, 6);
  portEXIT_CRITICAL(&pktMux);
  if (!known) return;

  if (!esp_now_is_peer_exist(mac)) {
    // Plaintext peer. With ESPNOW_ENCRYPT the controller is already registered as
    // an encrypted peer in setupEspNow (and onRecv only accepts that MAC).
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return;
  }

  uint8_t flags = 0;
  if (gate.linkLatch && !testMode) flags |= STATUS_FLAG_LINK_LATCH;
  if (testMode)                    flags |= STATUS_FLAG_TEST_MODE;
  if (!encReady)                   flags |= STATUS_FLAG_ENC_DOWN;
  StatusPacket sp = statusPacketMake(seq++, faultMask, flags);
  esp_now_send(mac, (const uint8_t*)&sp, sizeof(sp));
}

// ---------------- Loop watchdog ----------------
static bool loopWdtArmed = false;   // re-warned every second from loop() if false
// Subscribe the Arduino loop task to the task watchdog with a LOOP_WDT_MS timeout.
// The control tick feeds it. If loop() hangs, the ISR hook (wdt_hook.cpp) drops
// every direction pin LOW and the panic reboots the chip; the LEDC duty alone
// would otherwise keep the motors running through the hang. The core-0 idle check
// the framework enables (sdkconfig CHECK_IDLE_TASK_CPU0) is kept, at this timeout.
static void setupLoopWatchdog() {
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms     = LOOP_WDT_MS;
  cfg.idle_core_mask = 1u << 0;
  cfg.trigger_panic  = true;
  esp_err_t e = esp_task_wdt_reconfigure(&cfg);
  if (e == ESP_ERR_INVALID_STATE) e = esp_task_wdt_init(&cfg);  // core didn't start it
  if (e == ESP_OK) e = esp_task_wdt_add(NULL);                  // NULL = this task (loopTask)
  loopWdtArmed = (e == ESP_OK);
  if (loopWdtArmed) Serial.printf("loop watchdog armed (%lu ms)\n", (unsigned long)LOOP_WDT_MS);
  else              Serial.printf("WARN loop watchdog NOT armed: %s\n", esp_err_to_name(e));
}

// ---------------- main ----------------
void setup() {
  // Motor outputs FIRST: between reset and here the driver inputs float. Driving
  // INA/INB LOW and the PWM to 0 before the serial banner and its 200 ms delay
  // keeps that window as short as the boot itself.
  uint32_t dirLo = 0, dirHi = 0;
  for (uint8_t i = 0; i < 4; i++) {
    const Motor& m = motors[i];
    pinMode(m.inA, OUTPUT);
    pinMode(m.inB, OUTPUT);
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, LOW);
    ledcAttach(m.pwm, PWM_FREQ, PWM_RES);
    ledcWrite(m.pwm, 0);
    for (uint8_t pin : { m.inA, m.inB }) {
      if (pin < 32) dirLo |= 1u << pin;
      else          dirHi |= 1u << (pin - 32);
    }
  }
  g_motorDirMaskLo = dirLo;   // for the watchdog ISR hook (wdt_hook.cpp)
  g_motorDirMaskHi = dirHi;

  // TX ring buffer (must precede begin). The default is none, so a log line longer
  // than the 128-byte UART FIFO blocks loop() until its tail drains at 115200 —
  // ~4-6 ms per TLM/status line, inside a 10 ms control tick. TLM peaks near
  // 4 kB/s against an 11.5 kB/s drain, so the buffer never fills.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(200);
  Serial.println("\nmecanum robot: ESP-NOW + mecanum kinematics");
  Serial.printf("reset reason: %s\n", resetReasonStr(esp_reset_reason()));

  // All four encoders must come up: a wheel that reads 0 ticks would be driven to
  // full PWM by its PI loop. loop() holds the motors stopped while encReady is false.
  // The PCNT driver requests the internal pull-up on every encoder pin; GPIO34-39
  // are input-only and have none, so the gpio driver logged 8 harmless "GPIO
  // number error" lines at boot. Mute that tag just for this block; each unit's
  // setup result is still checked (encReady).
  const esp_log_level_t gpioLog = esp_log_level_get("gpio");
  esp_log_level_set("gpio", ESP_LOG_NONE);
  encReady = true;
  for (uint8_t i = 0; i < 4; i++) encReady = setupEncoder(i) && encReady;
  esp_log_level_set("gpio", gpioLog);

  setupEspNow();
  pidReset();
  setupLoopWatchdog();
}

static uint32_t nextTickUs  = 0;      // fixed-rate control schedule (micros)
static uint32_t lastTickUs  = 0;      // actual time of the previous tick, for dt
static bool     tickStarted = false;
static uint32_t lastLogMs   = 0;
static uint32_t lastSeen    = 0;

static void stopDrive() {
  if (!wasStopped) { motorStopAll(); pidReset(); wasStopped = true; }
}

void loop() {
  uint32_t now = millis();
  pollSerial();
  sendStatus(now);

  // Surface a dead radio. The one-shot setup line scrolls away and the robot is
  // headless, so re-warn periodically — visible whenever serial is attached.
  // Motors stay safe meanwhile: no packets arrive, so the watchdog holds stop.
  static uint32_t lastEspWarnMs = 0;
  if (!espNowReady && now - lastEspWarnMs >= 1000) {
    lastEspWarnMs = now;
    Serial.println("WARN ESP-NOW init failed — no radio link (motors held stopped)");
  }
  static uint32_t lastEncWarnMs = 0;
  if (!encReady && now - lastEncWarnMs >= 1000) {
    lastEncWarnMs = now;
    Serial.println("WARN encoder PCNT init failed — drive disabled (motors held stopped)");
  }
  // A failed watchdog arm leaves driving allowed (the kart works, the hang
  // protection is what's missing), so it re-warns like the others instead of
  // scrolling away after one line.
  static uint32_t lastWdtWarnMs = 0;
  if (!loopWdtArmed && now - lastWdtWarnMs >= 1000) {
    lastWdtWarnMs = now;
    Serial.println("WARN loop watchdog NOT armed — no hang protection");
  }

  // Radio e-stop during a bench test (flagged by onRecv): abort test mode, stopped.
  if (radioEstopReq) {
    radioEstopReq = false;
    if (testMode) {
      exitTestMode();
      Serial.println("TEST ABORT: radio e-stop");
    }
  }

  // Test-link watchdog: test mode feeds the radio watchdog itself, so without this
  // a `t`/`m` command ran forever after a crashed script or a pulled cable.
  if (testMode && linkAgeMs(now, lastSerialLineMs) > TEST_LINK_MS && testDriving()) {
    for (int i = 0; i < 4; i++) slotPwm[i] = 0;
    setPacketFromTest(0, 0, 0);
    motorStopAll();
    pidReset();
    wasStopped = true;
    Serial.println("WARN test link lost (no serial line) - stopped; bench tools send `k` as keepalive");
  }

  uint32_t nowUs = micros();
  if (!tickStarted || (int32_t)(nowUs - nextTickUs) >= 0) {
    // True elapsed time since the previous tick (us resolution; millis() quantized
    // dt to whole ms, a 10% error on a 10 ms tick).
    float dtMeas = tickStarted ? (float)(uint32_t)(nowUs - lastTickUs) * 1e-6f
                               : (float)CTRL_PERIOD_US * 1e-6f;
    if (dtMeas <= 0.0f) dtMeas = 0.01f;
    float dt = dtMeas;
    tickStarted = true;
    lastTickUs  = nowUs;
    // Fixed rate: the next tick is due one period after this one was DUE, so loop
    // latency doesn't accumulate into drift. More than a period behind -> restart
    // the schedule from now instead of bursting catch-up ticks.
    nextTickUs += CTRL_PERIOD_US;
    if ((int32_t)(nowUs - nextTickUs) >= 0) nextTickUs = nowUs + CTRL_PERIOD_US;
    esp_task_wdt_reset();   // the control tick is what the loop watchdog guards

    // (BUG-009) Clamp the CONTROL dt to 50 ms (5x nominal). A delayed tick would
    // otherwise inflate dt and simultaneously enlarge the CMD_SLEW step, PWM_SLEW
    // step, PID integral step and body integral step — one coordinated control
    // lurch, worst under payload where errors are large. Speed is still measured
    // over dtMeas, the real time the encoder delta spans (pidStep).
    if (dt > 0.05f) dt = 0.05f;

    CtrlPacket p;
    uint32_t age;
    portENTER_CRITICAL(&pktMux);
    // testMode keeps the serial-injected packet fresh so the radio watchdog can't
    // fire (the test-link watchdog above covers it). Downstream pipeline is
    // identical to the ESP-NOW path.
    if (testMode) lastPacketMs = now;
    p   = lastPacket;
    age = linkAgeMs(now, lastPacketMs);   // onRecv may stamp after `now` was read
    portEXIT_CRITICAL(&pktMux);

    // Drive gate (safety.h): one decision for "may the motors run", the link latch
    // (after a link loss, the first frame obeyed must be neutral) and the wheel-
    // fault latch (a switched-off wheel is re-armed after FAULT_CLEAR_MS of
    // neutral; the other wheels drive on meanwhile). Neutral = zero twist AND no
    // direct-mode PWM.
    bool directPwm = false;
    if (testMode && testSrc == TS_DIRECT)
      for (int i = 0; i < 4; i++) if (slotPwm[i] != 0) directPwm = true;
    DriveGateIn gin;
    gin.encReady = encReady;
    gin.linkOk   = age <= WATCHDOG_MS;
    gin.estop    = (p.flags & CTRL_FLAG_ESTOP) != 0;
    gin.neutral  = p.vx == 0 && p.vy == 0 && p.omega == 0 && !directPwm;
    gin.testMode = testMode;
    DriveGateOut gout = driveGateStep(&gate, gin, now, FAULT_CLEAR_MS);
    if (gout.faultCleared)
      for (int i = 0; i < 4; i++) rearmWheel(i, now, "sticks neutral");
    if (gout.linkLatched && !wasStopped)
      Serial.println("LINK LOST - stopped; centre the sticks to resume once the link is back");
    if (gout.linkReleased && !testMode)
      Serial.println("link ready (neutral frame received)");

    if (gout.stop) {
      stopDrive();
    } else if (testMode && testSrc == TS_DIRECT) {
      pidReset();   // first: it zeroes lastOutPwm, which then records the real output
      for (int i = 0; i < 4; i++) {
        motorWrite(i, slotPwm[i]);
        lastOutPwm[i] = (float)slotPwm[i];   // telemetry pwm= and the m -> t handoff
        lastTargetTps[i] = 0;
      }
      wasStopped = false;
    } else {
      // Field control: mirror the controller's selected mode (DISABLE bits) into
      // the runtime flags so the operator's live selection takes effect. Only on
      // the ESP-NOW path: in testMode the serial g/c/b commands own the flags (and
      // ESP-NOW is ignored in onRecv anyway). flags==0 (headset / legacy sender)
      // clears all DISABLE bits -> every feature ON -> unchanged behaviour.
      if (!testMode) {
        enableGovernor   = !(p.flags & CTRL_FLAG_GOV_OFF);
        enableClosedLoop = !(p.flags & CTRL_FLAG_CL_OFF);
        enableBodyLoop   = !(p.flags & CTRL_FLAG_BODY_OFF);
      }

      // Resuming from a stopped state (watchdog/estop release): the wheels may
      // have been nudged by hand while pidStep was skipped, so pid[].lastCount is
      // stale. Re-baseline it to the current count so the first velocity estimate
      // spans one tick, not the whole stopped interval (no recovery lurch).
      if (wasStopped) {
        for (int i = 0; i < 4; i++) pid[i].lastCount = readCount(i);
        // (BUG-012) Resume conservatively, not at full authority. pidReset() left
        // govScale=1.0; starting the post-watchdog ramp ungoverned means the first
        // ticks command a full mecanumMix with no cross-wheel protection (a surge/
        // yaw lurch). Start at the floor and let GOV_SLEW_UP earn speed back as the
        // wheels re-measure, so recovery is governed from the first tick.
        govScale = GOV_FLOOR;
      }

      // Base twist -> wheel targets. Slew-limit toward target to cap simultaneous
      // inrush current, moving all four wheels along one straight line so the
      // twist keeps its direction mid-ramp (kinematics.h slewQuad). The body-loop
      // correction below deliberately BYPASSES this slew (it is near-zero net
      // current and must stay responsive).
      int32_t cmd[4];
      mecanumMix(p.vx, p.vy, p.omega, cmd);
      int32_t maxStep = (int32_t)(CMD_SLEW * dt);
      if (maxStep < 1) maxStep = 1;
      slewQuad(curCmd, cmd, maxStep);

      // The twist the slewed base ACTUALLY expresses. It differs from the packet
      // whenever the mix normalized (forward+turn past 1000) or the ramp is still
      // running, and it is what both outer loops must judge against (BUG-010 for
      // the governor relax; the body loop below for the same reason).
      int32_t bVx, bVy, bW;
      mixInverse(curCmd, &bVx, &bVy, &bW);

      // Cross-wheel governor: scale every wheel by the worst-tracking one so the
      // group slows to match a held/stalled/supply-limited wheel instead of
      // yawing about it. Uses last tick's measured speeds; low-passed via GOV_SLEW
      // so a transient accel lag can't collapse drive. govScale eases back to 1.0
      // as the lagging wheel recovers. Judged on the BASE twist (curCmd).
      int32_t driveCmd[4];
      if (enableGovernor) {
        // A wheel with no usable feedback (open-loop, or switched off by a drive
        // fault) must not drag the group: skip it.
        bool valid[4];
        for (int i = 0; i < 4; i++) valid[i] = !openLoop[i] && !(faultMask & (1u << i));
        // Judge wheels by magnitude + output saturation (sign-independent), so a
        // held wheel is caught identically in forward and reverse. Uses last tick's
        // measured speeds and PWM.
        float gTarget = speedGovernorScale(curCmd, lastMeasTps, lastOutPwm,
                                           cmdRefTps(), (float)PWM_MAX,
                                           GOV_FLOOR, GOV_SAT_FRAC, valid);  // uniform ref
        // Spin-in-place scrubs all wheels equally below the no-load refTps, which the
        // governor would read as universal failure and throttle to a crawl. Symmetric
        // load is not the held-corner case it exists for, so fade the throttle by how
        // rotational the command is: pure spin -> governor off, translation unchanged.
        // (BUG-010) Judged on the slewed twist (bVx..), not the raw packet: on a snap
        // forward->turn the raw p.omega would over-relax a base whose omega is still
        // ramping in.
        gTarget = governorRotationRelax(gTarget, (int16_t)bVx, (int16_t)bVy, (int16_t)bW,
                                        GOV_SPIN_RELAX, GOV_SPIN_RELAX_TRANS_W);
        // Asymmetric slew: fall fast (catch a block), rise slow (smooth resume).
        float gDown = GOV_SLEW_DOWN * dt;
        float gUp   = GOV_SLEW_UP   * dt;
        if      (gTarget < govScale - gDown) govScale -= gDown;
        else if (gTarget > govScale + gUp)   govScale += gUp;
        else                                  govScale  = gTarget;
      } else {
        govScale = 1.0f;
      }
      for (int i = 0; i < 4; i++) driveCmd[i] = (int32_t)lroundf(curCmd[i] * govScale);

      // Body-space outer loop: estimate the chassis twist from the wheels and add
      // a small heading/centre correction the per-wheel loops structurally cannot
      // (a SISO wheel loop can't tell "one wheel lagging" from "the chassis
      // yawing"). The correction is an additive wheel-space twist added AFTER the
      // governor scale, so its yaw authority survives while the governor throttles
      // base magnitude. See body_loop.h.
      // With ONE wheel switched off the loop keeps running on the three live
      // wheels (that wheel's speed is reconstructed from them, kinematics.h
      // fillDeadWheel), so heading hold keeps compensating for the dead corner.
      // With two or more off the twist is underdetermined: the loop stands down.
      const int nDead = deadWheelCount();
      if (enableBodyLoop && nDead <= 1) {
        const float refTps = cmdRefTps();
        float meas[4] = { lastMeasTps[0], lastMeasTps[1], lastMeasTps[2], lastMeasTps[3] };
        if (nDead == 1)
          for (int i = 0; i < 4; i++) if (faultMask & (1u << i)) fillDeadWheel(meas, i);
        float vx_m, vy_m, w_m, s_m;
        forwardKinematics(meas, refTps, &vx_m, &vy_m, &w_m, &s_m);
        // Single-pole IIR on the body estimate (heavier on noisy omega/vy).
        bodyVxF += BODY_IIR_ALPHA_TRANS * (vx_m - bodyVxF);
        bodyVyF += BODY_IIR_ALPHA_W     * (vy_m - bodyVyF);
        bodyWF  += BODY_IIR_ALPHA_W     * (w_m  - bodyWF);
        bodySF  += BODY_IIR_ALPHA_W     * (s_m  - bodySF);

        // Handoff gating: freeze the outer integral whenever the governor owns the
        // situation — a saturated wheel, an active throttle, or high slip — so the
        // two loops never fight for magnitude authority.
        bool anySat = false;
        for (int i = 0; i < 4; i++) {
          float a = lastOutPwm[i] < 0 ? -lastOutPwm[i] : lastOutPwm[i];
          if (a >= GOV_SAT_FRAC * PWM_MAX) { anySat = true; break; }
        }
        float aS = bodySF < 0 ? -bodySF : bodySF;
        bool freezeI = anySat || govScale < BODY_GOV_FREEZE || aS > BODY_SLIP_GATE;

        BodyLoopCfg bc = { BODY_KP_TRANS, BODY_KP_W, BODY_KI_TRANS, BODY_KI_W,
                           BODY_I_MAX, BODY_W_THRESH, BODY_RATE, BODY_CORR_MAX,
                           BODY_YAW_FWD_FRAC, BODY_I_DECAY };
        // Reference = the twist the slewed base expresses (bVx..), NOT the packet.
        // With the packet, any forward+turn past 1000 left a standing error the
        // wheels could never close (the mix had scaled the command down): the loop
        // pinned a +120..+170 forward correction and, through the renormalize below,
        // took 6-14% of the turn away. It also chased the ramp on every start.
        // Throttled forward authority = commanded translation faded by the governor.
        // Bounds the yaw correction relative to the forward it rides on (BUG-004/005).
        float fwdAuth = (float)(abs(bVx) + abs(bVy)) * govScale;
        float dvx, dvy, dw;
        bodyCorrection((float)bVx, (float)bVy, (float)bW,
                       bodyVxF, bodyVyF, bodyWF, freezeI, govScale, fwdAuth, dt,
                       &bc, &bodyState, &dvx, &dvy, &dw);
        lastBodyCorr[0] = dvx; lastBodyCorr[1] = dvy; lastBodyCorr[2] = dw;

        // Additive correction twist -> wheel deltas, added AFTER governor scaling.
        int32_t corr[4];
        mecanumMix((int16_t)lroundf(dvx), (int16_t)lroundf(dvy),
                   (int16_t)lroundf(dw), corr);
        for (int i = 0; i < 4; i++) driveCmd[i] += corr[i];
        // Re-normalize the combined base+correction back to a valid <=1000 mecanum
        // set so the sum never over-commands a single wheel into pidStep (BUG-007).
        normalizeQuad(driveCmd, 1000);
      } else {
        bodyLoopReset();   // re-enable later starts clean, no stale correction
      }

      for (int i = 0; i < 4; i++) lastDriveCmd[i] = driveCmd[i];
      uint8_t tripped = pidStep(driveCmd, dt, dtMeas);
      wasStopped = false;
      if (tripped) {
        // Switch off ONLY the tripped wheel(s), right now (pidStep already wrote
        // this tick's PWM); the rest keep driving. Reported to the controller.
        driveGateTrip(&gate, now);
        faultMask |= tripped;
        char names[40] = "";
        uint32_t retryMs = 0;
        for (int i = 0; i < 4; i++) {
          if (!(tripped & (1u << i))) continue;
          motorWrite(i, 0);
          lastOutPwm[i]   = 0.0f;
          pid[i].integral = 0.0f;
          wheelRecoveryTrip(&recov[i], now, RECOV_CFG);
          retryMs = recov[i].delayMs;
          char one[16];
          snprintf(one, sizeof(one), "%s%s (slot %d)", names[0] ? ", " : "", RIDER_NAME[i], i);
          strncat(names, one, sizeof(names) - strlen(names) - 1);
        }
        Serial.printf("FAULT %s: driven at >=%.0f%% PWM with no encoder motion for %.0f ms "
                      "(dead encoder or jammed wheel) - wheel switched off, driving on the "
                      "others. Retry in %lu ms (sooner if it rolls freely, or release the sticks).\n",
                      names, FAULT_PWM_FRAC * 100.0f, FAULT_MS, (unsigned long)retryMs);
      }

      // Bring switched-off wheels back while driving: at once if the unpowered
      // wheel is seen rolling freely, else when its retry delay runs out.
      for (int i = 0; i < 4; i++) {
        if (!(faultMask & (1u << i))) continue;
        int why = wheelRecoveryStep(&recov[i], now, lastMeasTps[i], dt * 1000.0f, RECOV_CFG);
        if (why == 1) rearmWheel(i, now, "rolling freely");
        else if (why == 2) rearmWheel(i, now, "retry");
      }
    }
  }

  if (testMode) {
    if (now - lastTlmMs >= 50) {
      lastTlmMs = now;
      emitTlm(now);
    }
    return;
  }

  if (now - lastLogMs >= 500) {
    lastLogMs = now;
    CtrlPacket p;
    uint32_t age;
    portENTER_CRITICAL(&pktMux);
    p   = lastPacket;
    age = linkAgeMs(now, lastPacketMs);
    portEXIT_CRITICAL(&pktMux);
    bool fresh = age < 500 && p.seq != lastSeen;
    char state[200], batt[24];
    fmtCtrlState(state, sizeof(state));
    fmtBatt(batt, sizeof(batt));
    char stall[20] = "";
    if (wheelStalled[0] || wheelStalled[1] || wheelStalled[2] || wheelStalled[3])
      snprintf(stall, sizeof(stall), " STALL=[%d %d %d %d]",
               wheelStalled[0], wheelStalled[1], wheelStalled[2], wheelStalled[3]);
    Serial.printf("seq=%lu vx=%d vy=%d w=%d | %s meas=[%.0f %.0f %.0f %.0f] crcDrops=%lu foreign=%lu%s%s%s%s\n",
                  (unsigned long)p.seq, p.vx, p.vy, p.omega, state,
                  lastMeasTps[0], lastMeasTps[1], lastMeasTps[2], lastMeasTps[3],
                  (unsigned long)crcDrops, (unsigned long)foreignDrops, batt, stall,
                  gate.fault ? " FAULT" : (gate.linkLatch ? " LINK-LATCHED" : ""),
                  fresh ? "" : " (stale)");
    lastSeen = p.seq;
  }
}
