#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>   // esp_reset_reason()
#include "protocol.h"
#include "config_robot.h"
#include "kinematics.h"     // mecanumMix() + forwardKinematics()
#include "governor.h"       // speedGovernorScale() — cross-wheel sync
#include "body_loop.h"      // bodyCorrection() — body-space outer loop
#include "control_math.h"   // crc8()

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

static volatile long encCount[4] = { 0, 0, 0, 0 };
// Derived from solo-PWM test: +PWM raw_tps signs [-, +, -, +] for [FL,FR,RL,RR].
static const int8_t encSign[4]   = { -1, +1, -1, +1 };

// Per-wheel open-loop override: pure feed-forward PWM, no P/I (for a dead
// encoder). All four encoders work now, so all closed-loop. Kept as a knob:
// flip an entry true if that encoder fails, so its wheel still drives.
static const bool openLoop[4] = { false, false, false, false };

// Full 4x quadrature decode (BUG-008 fixed 2026-06-02). Interrupt on CHANGE of
// BOTH channels; each A/B edge advances a 2-bit state and a transition lookup
// adds +1/-1 (or 0 for no-change / illegal double-step). This rejects the phantom
// counts the old 1x decode streamed when a held wheel dithered encA: a real
// rotation walks the Gray sequence (net +/-), but dither bounces between two
// states and nets ~0. Sign convention is preserved (00->10 == +1, the same edge
// the old code counted +1 on), so encSign[] stays valid — see [[wheel-sync-and-encsign]].
// Tick magnitude is ~4x the old rate, so MAX_TPS[] must be recalibrated.
static volatile uint8_t encState[4] = { 0, 0, 0, 0 };  // last (A<<1)|B per wheel
// Index = (oldState<<2)|newState, state=(A<<1)|B. +1 along the increment Gray
// cycle 00->10->11->01->00, -1 reverse, 0 for no-move and illegal both-bit jumps.
static const int8_t QTAB[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0,
};
static inline void IRAM_ATTR encUpdate(uint8_t i) {
  uint8_t s   = (uint8_t)((digitalRead(motors[i].encA) << 1) | digitalRead(motors[i].encB));
  uint8_t idx = (uint8_t)((encState[i] << 2) | s);
  encCount[i] += QTAB[idx];
  encState[i]  = s;
}
static void IRAM_ATTR encISR0() { encUpdate(0); }
static void IRAM_ATTR encISR1() { encUpdate(1); }
static void IRAM_ATTR encISR2() { encUpdate(2); }
static void IRAM_ATTR encISR3() { encUpdate(3); }
static void (*encISRs[4])() = { encISR0, encISR1, encISR2, encISR3 };

static bool isInputOnly(uint8_t pin) {
  return pin == 34 || pin == 35 || pin == 36 || pin == 39;
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

// Body-space outer loop: persistent state + IIR-filtered chassis-twist estimate.
static BodyLoopState bodyState = { 0, 0, 0, 0, 0, 0 };
static float bodyVxF = 0, bodyVyF = 0, bodyWF = 0, bodySF = 0;  // filtered vx,vy,omega,slip
static float lastBodyCorr[3] = { 0, 0, 0 };                     // last (dvx,dvy,dw) for telemetry

// Last PID step values for telemetry (M1 only, to keep log short)
static float lastTargetTps[4] = { 0, 0, 0, 0 };
static float lastMeasTps[4]   = { 0, 0, 0, 0 };
static float lastOutPwm[4]    = { 0, 0, 0, 0 };

// Per-wheel stall detection (telemetry): pinned near max PWM but barely moving.
static float stallMs[4]      = { 0, 0, 0, 0 };
static bool  wheelStalled[4] = { false, false, false, false };

static long readCountAtomic(uint8_t i) {
  noInterrupts();
  long c = encCount[i];
  interrupts();
  return c;
}

static void pidReset() {
  for (int i = 0; i < 4; i++) {
    pid[i].integral  = 0.0f;
    pid[i].lastCount = readCountAtomic(i);
    curCmd[i]        = 0;     // drop the slew ramp so resume starts from rest
    lastOutPwm[i]    = 0.0f;  // output slew restarts from zero too
    stallMs[i]       = 0.0f;
    wheelStalled[i]  = false;
  }
  govScale = 1.0f;            // resume ungoverned, ramp in again from measurement
  // Body-space outer loop: drop integrals, filtered estimate, and correction so
  // resume starts from a clean chassis state (no stale heading bias).
  bodyState.ix = bodyState.iy = bodyState.iw = 0.0f;
  bodyState.dvx = bodyState.dvy = bodyState.dw = 0.0f;
  bodyVxF = bodyVyF = bodyWF = bodySF = 0.0f;
  lastBodyCorr[0] = lastBodyCorr[1] = lastBodyCorr[2] = 0.0f;
}

// cmd[i] in [-1000..+1000]; dt in seconds.
static void pidStep(const int32_t cmd[4], float dt) {
  for (int i = 0; i < 4; i++) {
    // Signed encoder delta (apply sign to fix wiring inversions).
    long now   = readCountAtomic(i);
    long delta = now - pid[i].lastCount;
    pid[i].lastCount = now;

    // Glitch rejection: a real wheel can't exceed ~MAX_TPS, so a larger
    // per-tick delta is electrical noise from motor current transients. Clamp
    // it so the loop tracks real motion, not the spike. Without this, inrush
    // noise spikes the velocity estimate, the PID slams PWM to react, and the
    // current swing makes more noise — a self-sustaining limit cycle (worst on
    // vx+, all 4 motors inrushing forward together).
    long maxDelta = (long)(MAX_TPS[i] * 1.5f * dt) + 2;
    if (delta >  maxDelta) delta =  maxDelta;
    if (delta < -maxDelta) delta = -maxDelta;
    float measuredTps = encSign[i] * (float)delta / dt;

    // Feed-forward is per-wheel (a weak-battery wheel needs more PWM per tick/sec),
    // but the TARGET is a UNIFORM absolute speed capped at the weakest wheel
    // (refTps = maxTpsMin) so cmd 1000 means the same ground speed on every wheel
    // and the cart drives straight. A strong wheel simply uses less of its range.
    const float refTps    = maxTpsMin();
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
      float prevOut    = lastOutPwm[i];
      float maxPwmStep = PWM_SLEW * dt;
      float out        = 0.0f;
      if (out > prevOut + maxPwmStep) out = prevOut + maxPwmStep;
      if (out < prevOut - maxPwmStep) out = prevOut - maxPwmStep;
      motorWrite(i, (int16_t)lroundf(out));
      stallMs[i]       = 0.0f;
      wheelStalled[i]  = false;
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
      lastTargetTps[i] = targetTps;
      lastMeasTps[i]   = 0.0f;
      lastOutPwm[i]    = out;
      continue;
    }

    float err = targetTps - measuredTps;

    // Compute the desired output from the CURRENT integral, then apply BOTH
    // actuator limits — the hard PWM_MAX clamp and the per-tick slew limiter —
    // to get the value actually delivered this tick.
    float prevOut    = lastOutPwm[i];
    float maxPwmStep = PWM_SLEW * dt;
    float desired    = kff * targetTps + Kp * err + pid[i].integral;

    float out = desired;
    if (out >  PWM_MAX) out =  PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;
    // Output slew-rate limit: cap per-tick PWM change so the loop can't slam
    // the motor rail-to-rail (kills the rotate-start oscillation).
    if (out > prevOut + maxPwmStep) out = prevOut + maxPwmStep;
    if (out < prevOut - maxPwmStep) out = prevOut - maxPwmStep;

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
    lastTargetTps[i] = targetTps;
    lastMeasTps[i]   = measuredTps;
    lastOutPwm[i]    = out;
  }
}

// ---------------- Test mode (serial command driven) ----------------
// testMode + testSrc selects what feeds the drive step:
//   TS_MIX:    `t` command -> mecanumMix -> pidStep -> motorWrite (production path).
//   TS_DIRECT: `m` command -> motorWrite (bypass kinematics + PID, raw PWM per slot).
// Commands:
//   t <vx> <vy> <omega>     mix path (each -1000..+1000)
//   m <slot> <pwm>          direct path (slot 0..3, pwm -1023..+1023)
//   s                       stop (zero everything)
//   r                       zero encoder counters + PID
//   x                       exit test mode (ESP-NOW control resumes)
//   ?                       print one-shot status
enum TestSrc : uint8_t { TS_MIX, TS_DIRECT };
static bool      testMode = false;
static TestSrc   testSrc  = TS_MIX;
static int16_t   slotPwm[4] = { 0, 0, 0, 0 };
static char      cmdBuf[64];
static uint8_t   cmdLen = 0;

static long      prevEnc[4]  = { 0, 0, 0, 0 };
static uint32_t  prevTlmMs   = 0;
static uint32_t  lastTlmMs   = 0;

static int16_t clampCmd(int v) { return (int16_t)constrain(v, -1000, 1000); }
static int16_t clampPwm(int v) { return (int16_t)constrain(v, -PWM_MAX, PWM_MAX); }

// Forward decls — defined below near ESP-NOW state.
static void setPacketFromTest(int16_t vx, int16_t vy, int16_t omega);
static void getPacketSnapshot(CtrlPacket& out);

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
      noInterrupts();
      for (int i = 0; i < 4; i++) encCount[i] = 0;
      interrupts();
      pidReset();
      // Baseline telemetry from the post-zero counts (not a hard 0) so the first
      // TLM delta after `r` doesn't show a phantom velocity from ISR ticks that
      // landed between the zeroing and pidReset.
      for (int i = 0; i < 4; i++) prevEnc[i] = readCountAtomic(i);
      Serial.println("OK r");
      break;
    case 'x':
      testMode = false;
      testSrc  = TS_MIX;
      for (int i = 0; i < 4; i++) slotPwm[i] = 0;
      setPacketFromTest(0, 0, 0);
      motorStopAll();
      pidReset();
      Serial.println("OK x");
      break;
    case '?': {
      CtrlPacket p;
      getPacketSnapshot(p);
      Serial.printf("STATUS testMode=%d src=%s packet vx=%d vy=%d omega=%d slotPwm=[%d %d %d %d]\n",
                    testMode ? 1 : 0, testSrc == TS_DIRECT ? "DIRECT" : "MIX",
                    p.vx, p.vy, p.omega,
                    slotPwm[0], slotPwm[1], slotPwm[2], slotPwm[3]);
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
      if (cmdLen > 0) handleCommand(cmdBuf);
      cmdLen = 0;
      continue;
    }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = (char)c;
  }
}

static void emitTlm(uint32_t now) {
  // Skip a same-millisecond re-entry: dividing by a 1 ms floor inflates raw_tps
  // ~50x and shows a phantom spike on the readout used to diagnose stalls.
  if (prevTlmMs != 0 && now == prevTlmMs) return;
  uint32_t dt_ms = (prevTlmMs == 0) ? 50 : (now - prevTlmMs);
  float dt = dt_ms / 1000.0f;
  long cnt[4], delta[4];
  for (int i = 0; i < 4; i++) {
    cnt[i]    = readCountAtomic(i);
    delta[i]  = cnt[i] - prevEnc[i];
    prevEnc[i] = cnt[i];
  }
  prevTlmMs = now;

  CtrlPacket p;
  getPacketSnapshot(p);
  int32_t cmd[4];
  mecanumMix(p.vx, p.vy, p.omega, cmd);

  char batt[24] = "";
  float vb = readBattVolts();
  if (vb >= 0) snprintf(batt, sizeof(batt), " vbat=%.2f", vb);

  Serial.printf("TLM ms=%lu cmd=[%ld %ld %ld %ld] gov=%.2f body=[%.0f %.0f %.0f] s=%.0f corr=[%.0f %.0f %.0f] pwm=[%.0f %.0f %.0f %.0f] raw_tps=[%.1f %.1f %.1f %.1f] cnt=[%ld %ld %ld %ld]%s\n",
                (unsigned long)now,
                (long)cmd[0], (long)cmd[1], (long)cmd[2], (long)cmd[3],
                govScale,
                bodyVxF, bodyVyF, bodyWF, bodySF,
                lastBodyCorr[0], lastBodyCorr[1], lastBodyCorr[2],
                lastOutPwm[0], lastOutPwm[1], lastOutPwm[2], lastOutPwm[3],
                (float)delta[0]/dt, (float)delta[1]/dt, (float)delta[2]/dt, (float)delta[3]/dt,
                cnt[0], cnt[1], cnt[2], cnt[3], batt);
}

// ---------------- ESP-NOW receive ----------------
static volatile uint32_t lastPacketMs = 0;
static CtrlPacket lastPacket = { 0, 0, 0, 0, 0, 0, 0 };
static portMUX_TYPE pktMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t crcDrops = 0;  // count of frames rejected on bad CRC
static bool espNowReady = false;        // false until radio init + recv cb succeed

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(CtrlPacket)) return;
  if (testMode) return;  // ignore ESP-NOW while serial-injected test packet is active

  CtrlPacket in;
  memcpy(&in, data, sizeof(in));

  // Integrity (#4): drop frames whose crc8 over the leading bytes mismatches.
  if (in.crc != crc8((const uint8_t*)&in, offsetof(CtrlPacket, crc))) {
    crcDrops++;
    return;
  }

  // Drop stale/duplicate/reordered frames: accept only a newer seq. The int32
  // cast makes the compare wrap-safe at 2^32. A large backward jump (>=1000)
  // means the controller rebooted (seq restarts near 0) — resync rather than
  // lock the link out, since lastSeq would otherwise reject every fresh frame.
  static uint32_t lastSeq = 0;
  int32_t d = (int32_t)(in.seq - lastSeq);
  if (d <= 0 && d > -1000) return;
  lastSeq = in.seq;

  portENTER_CRITICAL(&pktMux);
  lastPacket   = in;
  lastPacketMs = millis();
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

  Serial.print("Robot MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }

#if ESPNOW_ENCRYPT
  // Set the primary key, then register the controller as an encrypted peer so
  // the robot will accept (and only accept) encrypted frames from it (#3).
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

// ---------------- main ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nmecanum robot: ESP-NOW + mecanum kinematics");
  Serial.printf("reset reason: %s\n", resetReasonStr(esp_reset_reason()));

  for (uint8_t i = 0; i < 4; i++) {
    const Motor& m = motors[i];
    pinMode(m.inA, OUTPUT);
    pinMode(m.inB, OUTPUT);
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, LOW);
    ledcAttach(m.pwm, PWM_FREQ, PWM_RES);
    ledcWrite(m.pwm, 0);
    pinMode(m.encA, isInputOnly(m.encA) ? INPUT : INPUT_PULLUP);
    pinMode(m.encB, isInputOnly(m.encB) ? INPUT : INPUT_PULLUP);
    // 4x quadrature: seed the state from the rest level, then fire on every edge
    // of BOTH channels. Both pins share one ISR per wheel; same-core GPIO ISRs
    // serialize, so encState[i] needs no extra guard.
    encState[i] = (uint8_t)((digitalRead(m.encA) << 1) | digitalRead(m.encB));
    attachInterrupt(digitalPinToInterrupt(m.encA), encISRs[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(m.encB), encISRs[i], CHANGE);
  }

  setupEspNow();
  pidReset();
}

static uint32_t lastDriveMs = 0;
static uint32_t lastLogMs   = 0;
static uint32_t lastSeen    = 0;
static bool     wasStopped  = true;

void loop() {
  uint32_t now = millis();
  pollSerial();

  // Surface a dead radio. The one-shot setup line scrolls away and the robot is
  // headless, so re-warn periodically — visible whenever serial is attached.
  // Motors stay safe meanwhile: no packets arrive, so the watchdog holds stop.
  static uint32_t lastEspWarnMs = 0;
  if (!espNowReady && now - lastEspWarnMs >= 1000) {
    lastEspWarnMs = now;
    Serial.println("WARN ESP-NOW init failed — no radio link (motors held stopped)");
  }

  if (now - lastDriveMs >= 10) {
    float dt = (now - lastDriveMs) / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;
    // (BUG-009) Clamp dt to ~2x nominal. A delayed tick (the 500ms serial log or an
    // ESP-NOW callback burst runs in this same loop) would otherwise inflate dt and
    // simultaneously enlarge the CMD_SLEW step, PWM_SLEW step, PID integral step and
    // body integral step — one coordinated control lurch, worst under payload where
    // errors are large. The same unbounded dt also widened the encoder glitch clamp.
    if (dt > 0.05f) dt = 0.05f;
    lastDriveMs = now;

    CtrlPacket p;
    uint32_t age;
    portENTER_CRITICAL(&pktMux);
    // testMode keeps the serial-injected packet fresh so the watchdog can't
    // fire. Downstream pipeline is identical to the ESP-NOW path.
    if (testMode) lastPacketMs = now;
    p   = lastPacket;
    age = now - lastPacketMs;
    portEXIT_CRITICAL(&pktMux);

    if (age > WATCHDOG_MS || (p.flags & 0x01)) {
      if (!wasStopped) { motorStopAll(); pidReset(); wasStopped = true; }
    } else if (testMode && testSrc == TS_DIRECT) {
      for (int i = 0; i < 4; i++) {
        motorWrite(i, slotPwm[i]);
        lastOutPwm[i] = (float)slotPwm[i];
        lastTargetTps[i] = 0;
      }
      pidReset();
      wasStopped = false;
    } else {
      // Resuming from a stopped state (watchdog/estop release): the wheels may
      // have been nudged by hand while pidStep was skipped, so pid[].lastCount is
      // stale. Re-baseline it to the current count so the first velocity estimate
      // spans one tick, not the whole stopped interval (no recovery lurch).
      if (wasStopped) {
        for (int i = 0; i < 4; i++) pid[i].lastCount = readCountAtomic(i);
        // (BUG-012) Resume conservatively, not at full authority. pidReset() left
        // govScale=1.0; starting the post-watchdog ramp ungoverned means the first
        // ticks command a full mecanumMix with no cross-wheel protection (a surge/
        // yaw lurch). Start at the floor and let GOV_SLEW_UP earn speed back as the
        // wheels re-measure, so recovery is governed from the first tick.
        govScale = GOV_FLOOR;
      }

      // Base twist -> wheel targets. Slew-limit toward target to cap simultaneous
      // inrush current. The body-loop correction below deliberately BYPASSES this
      // slew (it is near-zero net current and must stay responsive).
      int32_t cmd[4];
      mecanumMix(p.vx, p.vy, p.omega, cmd);
      int32_t maxStep = (int32_t)(CMD_SLEW * dt);
      if (maxStep < 1) maxStep = 1;
      for (int i = 0; i < 4; i++) {
        int32_t d = cmd[i] - curCmd[i];
        if (d >  maxStep) d =  maxStep;
        if (d < -maxStep) d = -maxStep;
        curCmd[i] += d;
      }

      // Cross-wheel governor: scale every wheel by the worst-tracking one so the
      // group slows to match a held/stalled/supply-limited wheel instead of
      // yawing about it. Uses last tick's measured speeds; low-passed via GOV_SLEW
      // so a transient accel lag can't collapse drive. govScale eases back to 1.0
      // as the lagging wheel recovers. Judged on the BASE twist (curCmd).
      int32_t driveCmd[4];
#if SYNC_GOVERNOR
      bool valid[4] = { !openLoop[0], !openLoop[1], !openLoop[2], !openLoop[3] };
      // Judge wheels by magnitude + output saturation (sign-independent), so a
      // held wheel is caught identically in forward and reverse. Uses last tick's
      // measured speeds and PWM.
      float gTarget = speedGovernorScale(curCmd, lastMeasTps, lastOutPwm,
                                         maxTpsMin(), (float)PWM_MAX,
                                         GOV_FLOOR, GOV_SAT_FRAC, valid);  // uniform ref
      // Spin-in-place scrubs all wheels equally below the no-load refTps, which the
      // governor would read as universal failure and throttle to a crawl. Symmetric
      // load is not the held-corner case it exists for, so fade the throttle by how
      // rotational the command is: pure spin -> governor off, translation unchanged.
      // (BUG-010) Judge the relax on the SLEWED command (decoded from curCmd), not
      // the raw packet twist: the governor scale being faded was computed from the
      // slewed curCmd, so on a snap forward->turn the raw p.omega would over-relax a
      // base whose omega is still ramping in. Decode the body twist from curCmd.
      int16_t sVxC = (int16_t)((curCmd[0] + curCmd[1] + curCmd[2] + curCmd[3]) / 4);
      int16_t sVyC = (int16_t)((-curCmd[0] + curCmd[1] + curCmd[2] - curCmd[3]) / 4);
      int16_t sWC  = (int16_t)((-curCmd[0] + curCmd[1] - curCmd[2] + curCmd[3]) / 4);
      gTarget = governorRotationRelax(gTarget, sVxC, sVyC, sWC,
                                      GOV_SPIN_RELAX, GOV_SPIN_RELAX_TRANS_W);
      // Asymmetric slew: fall fast (catch a block), rise slow (smooth resume).
      float gDown = GOV_SLEW_DOWN * dt;
      float gUp   = GOV_SLEW_UP   * dt;
      if      (gTarget < govScale - gDown) govScale -= gDown;
      else if (gTarget > govScale + gUp)   govScale += gUp;
      else                                  govScale  = gTarget;
#else
      govScale = 1.0f;
#endif
      for (int i = 0; i < 4; i++) driveCmd[i] = (int32_t)lroundf(curCmd[i] * govScale);

      // Body-space outer loop: estimate the chassis twist from the wheels and add
      // a small heading/centre correction the per-wheel loops structurally cannot
      // (a SISO wheel loop can't tell "one wheel lagging" from "the chassis
      // yawing"). The correction is an additive wheel-space twist added AFTER the
      // governor scale, so its yaw authority survives while the governor throttles
      // base magnitude. See body_loop.h.
#if BODY_LOOP
      {
        const float refTps = maxTpsMin();
        float vx_m, vy_m, w_m, s_m;
        forwardKinematics(lastMeasTps, refTps, &vx_m, &vy_m, &w_m, &s_m);
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
        // Throttled forward authority = commanded translation faded by the governor.
        // Bounds the yaw correction relative to the forward it rides on (BUG-004/005).
        float aVx = p.vx < 0 ? -(float)p.vx : (float)p.vx;
        float aVy = p.vy < 0 ? -(float)p.vy : (float)p.vy;
        float fwdAuth = (aVx + aVy) * govScale;
        float dvx, dvy, dw;
        bodyCorrection((float)p.vx, (float)p.vy, (float)p.omega,
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
      }
#endif

      pidStep(driveCmd, dt);
      wasStopped = false;
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
    age = now - lastPacketMs;
    portEXIT_CRITICAL(&pktMux);
    bool fresh = age < 500 && p.seq != lastSeen;
    int32_t cmd[4];
    mecanumMix(p.vx, p.vy, p.omega, cmd);
    char batt[24] = "";
    float vb = readBattVolts();
    if (vb >= 0) snprintf(batt, sizeof(batt), " vbat=%.2f", vb);
    char stall[20] = "";
    if (wheelStalled[0] || wheelStalled[1] || wheelStalled[2] || wheelStalled[3])
      snprintf(stall, sizeof(stall), " STALL=[%d %d %d %d]",
               wheelStalled[0], wheelStalled[1], wheelStalled[2], wheelStalled[3]);
    Serial.printf("seq=%lu vx=%d vy=%d w=%d | cmd=[%ld %ld %ld %ld] gov=%.2f body=[%.0f %.0f %.0f] s=%.0f corr=[%.0f %.0f %.0f] pwm=[%.0f %.0f %.0f %.0f] meas=[%.0f %.0f %.0f %.0f] crcDrops=%lu%s%s%s\n",
                  (unsigned long)p.seq, p.vx, p.vy, p.omega,
                  (long)cmd[0], (long)cmd[1], (long)cmd[2], (long)cmd[3],
                  govScale,
                  bodyVxF, bodyVyF, bodyWF, bodySF,
                  lastBodyCorr[0], lastBodyCorr[1], lastBodyCorr[2],
                  lastOutPwm[0], lastOutPwm[1], lastOutPwm[2], lastOutPwm[3],
                  lastMeasTps[0], lastMeasTps[1], lastMeasTps[2], lastMeasTps[3],
                  (unsigned long)crcDrops, batt, stall,
                  fresh ? "" : " (stale)");
    lastSeen = p.seq;
  }
}
