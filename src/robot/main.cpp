#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"

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
// unchanged. NOTE: FR (slot 1) encoder reads 0 (dead wiring on GPIO 16/4) —
// hardware fix still pending; PID drives FR open-loop via feed-forward.
static const Motor motors[4] = {
  { 18,  5, 19, 13, 17 },  // FL  motor=(18, 5,19)  encoder=(13,17)
  { 21, 23, 22, 16,  4 },  // FR  motor=(21,23,22)  encoder=(16, 4) [encoder dead - HW]
  { 25, 27, 33, 39, 36 },  // RL  was old-slot3 hw=(25,33,27); inA<->inB swapped to invert  encoder=(39,36)
  { 26, 32, 14, 35, 34 },  // RR  was old-slot2 hw=(26,14,32); inA<->inB swapped to invert  encoder=(35,34)
};

static const int PWM_FREQ = 25000;
static const int PWM_RES  = 10;
static const int PWM_MAX  = (1 << PWM_RES) - 1;
static const int DEADBAND = 0;  // disabled: PID feed-forward handles low-speed PWM directly

static volatile long encCount[4] = { 0, 0, 0, 0 };
// Derived from solo-PWM test: +PWM raw_tps signs [-, +, -, +] for [FL,FR,RL,RR].
static const int8_t encSign[4]   = { -1, +1, -1, +1 };

// FR (slot 1) encoder is dead (HW wiring on GPIO 16/4). Run that wheel open-loop:
// pure feed-forward PWM, no P/I — otherwise the PID sees measured=0, computes a
// huge error and over-drives FR to saturation. Others stay closed-loop.
static const bool openLoop[4] = { false, true, false, false };

static void IRAM_ATTR encISR0() { if (digitalRead(motors[0].encB)) encCount[0]--; else encCount[0]++; }
static void IRAM_ATTR encISR1() { if (digitalRead(motors[1].encB)) encCount[1]--; else encCount[1]++; }
static void IRAM_ATTR encISR2() { if (digitalRead(motors[2].encB)) encCount[2]--; else encCount[2]++; }
static void IRAM_ATTR encISR3() { if (digitalRead(motors[3].encB)) encCount[3]--; else encCount[3]++; }
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

// ---------------- Mecanum kinematics ----------------
// Wheel command in [-1000..+1000] (units: thousandths of max wheel speed).
// X-pattern rollers, slot map: M1=FL, M2=FR, M3=RL, M4=RR.
static void mecanumMix(int16_t vx, int16_t vy, int16_t omega, int32_t outCmd[4]) {
  outCmd[0] = (int32_t)vx - vy - omega;  // FL
  outCmd[1] = (int32_t)vx + vy + omega;  // FR
  outCmd[2] = (int32_t)vx + vy - omega;  // RL
  outCmd[3] = (int32_t)vx - vy + omega;  // RR

  int32_t peak = 1000;
  for (int i = 0; i < 4; i++) if (abs(outCmd[i]) > peak) peak = abs(outCmd[i]);
  if (peak > 1000) {
    for (int i = 0; i < 4; i++) outCmd[i] = (outCmd[i] * 1000) / peak;
  }
}

// ---------------- Per-wheel PI velocity control ----------------
// Measured at ramp test: ~2130 ticks/sec at full PWM (1023). Use as cmd=1000 reference.
static const float MAX_TPS = 2100.0f;          // max wheel speed (ticks/sec) used to scale cmd
static const float Kff     = (float)PWM_MAX / MAX_TPS;  // ~0.487 — feed-forward gain
static const float Kp      = 0.15f;
static const float Ki      = 0.4f;             // re-enabled with conditional-integration anti-windup
static const float I_MAX   = 0.6f * (float)PWM_MAX;  // bound integral authority (~614)

struct PidState {
  float  integral;
  long   lastCount;
};
static PidState pid[4] = {};

// Command slew limiter: ramp each wheel cmd toward its mix target instead of
// stepping instantly. Four motors slamming 0->full at once draw near-stall
// current simultaneously (no back-EMF yet) and collapse a weak supply rail —
// worst during spin-in-place. Ramping staggers the current rise.
static const float CMD_SLEW = 4000.0f;  // cmd units/sec (full 0..1000 in ~250ms)
static int32_t curCmd[4] = { 0, 0, 0, 0 };

// Last PID step values for telemetry (M1 only, to keep log short)
static float lastTargetTps[4] = { 0, 0, 0, 0 };
static float lastMeasTps[4]   = { 0, 0, 0, 0 };
static float lastOutPwm[4]    = { 0, 0, 0, 0 };

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
    curCmd[i]        = 0;  // drop the slew ramp so resume starts from rest
  }
}

// cmd[i] in [-1000..+1000]; dt in seconds.
static void pidStep(const int32_t cmd[4], float dt) {
  for (int i = 0; i < 4; i++) {
    // Signed encoder delta (apply sign to fix wiring inversions).
    long now = readCountAtomic(i);
    float measuredTps = encSign[i] * (now - pid[i].lastCount) / dt;
    pid[i].lastCount  = now;

    float targetTps = ((float)cmd[i] / 1000.0f) * MAX_TPS;

    if (cmd[i] == 0) {
      pid[i].integral = 0.0f;
      motorWrite(i, 0);
      lastTargetTps[i] = 0;
      lastMeasTps[i]   = measuredTps;
      lastOutPwm[i]    = 0;
      continue;
    }

    // Dead-encoder wheels: open-loop feed-forward only. No error term (measured
    // is meaningless), so PWM tracks the commanded speed directly.
    if (openLoop[i]) {
      float out = Kff * targetTps;
      out = constrain(out, -(float)PWM_MAX, (float)PWM_MAX);
      motorWrite(i, (int16_t)out);
      pid[i].integral  = 0.0f;
      pid[i].lastCount = now;
      lastTargetTps[i] = targetTps;
      lastMeasTps[i]   = 0.0f;
      lastOutPwm[i]    = out;
      continue;
    }

    float err = targetTps - measuredTps;

    // Conditional-integration anti-windup: only accumulate when the output is
    // not already saturated in the direction the error would push it. Stops
    // the integral from running away while PWM is pinned (the old failure).
    float pre = Kff * targetTps + Kp * err + pid[i].integral;
    bool saturated = (pre >=  PWM_MAX && err > 0) ||
                     (pre <= -PWM_MAX && err < 0);
    if (!saturated) {
      pid[i].integral += err * dt * Ki;
      if (pid[i].integral >  I_MAX) pid[i].integral =  I_MAX;
      if (pid[i].integral < -I_MAX) pid[i].integral = -I_MAX;
    }

    float out = Kff * targetTps + Kp * err + pid[i].integral;
    if (out >  PWM_MAX) out =  PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;

    motorWrite(i, (int16_t)out);
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
      for (int i = 0; i < 4; i++) prevEnc[i] = 0;
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
  uint32_t dt_ms = (prevTlmMs == 0) ? 50 : (now - prevTlmMs);
  if (dt_ms == 0) dt_ms = 1;
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

  Serial.printf("TLM ms=%lu cmd=[%ld %ld %ld %ld] pwm=[%.0f %.0f %.0f %.0f] raw_tps=[%.1f %.1f %.1f %.1f] cnt=[%ld %ld %ld %ld]\n",
                (unsigned long)now,
                (long)cmd[0], (long)cmd[1], (long)cmd[2], (long)cmd[3],
                lastOutPwm[0], lastOutPwm[1], lastOutPwm[2], lastOutPwm[3],
                (float)delta[0]/dt, (float)delta[1]/dt, (float)delta[2]/dt, (float)delta[3]/dt,
                cnt[0], cnt[1], cnt[2], cnt[3]);
}

// ---------------- ESP-NOW receive ----------------
static volatile uint32_t lastPacketMs = 0;
static CtrlPacket lastPacket = { 0, 0, 0, 0, 0, 0 };
static portMUX_TYPE pktMux = portMUX_INITIALIZER_UNLOCKED;

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(CtrlPacket)) return;
  if (testMode) return;  // ignore ESP-NOW while serial-injected test packet is active

  // Drop stale/duplicate/reordered frames: accept only a newer seq. The int32
  // cast makes the compare wrap-safe at 2^32. A large backward jump (>=1000)
  // means the controller rebooted (seq restarts near 0) — resync rather than
  // lock the link out, since lastSeq would otherwise reject every fresh frame.
  static uint32_t lastSeq = 0;
  uint32_t incoming;
  memcpy(&incoming, data, sizeof(incoming));  // seq is the first field
  int32_t d = (int32_t)(incoming - lastSeq);
  if (d <= 0 && d > -1000) return;
  lastSeq = incoming;

  portENTER_CRITICAL(&pktMux);
  memcpy(&lastPacket, data, sizeof(CtrlPacket));
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
  esp_now_register_recv_cb(onRecv);
  Serial.println("ESP-NOW listening");
}

// ---------------- main ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nmecanum robot: ESP-NOW + mecanum kinematics");

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
    attachInterrupt(digitalPinToInterrupt(m.encA), encISRs[i], RISING);
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

  if (now - lastDriveMs >= 10) {
    float dt = (now - lastDriveMs) / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;
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

    if (age > 500 || (p.flags & 0x01)) {
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
      int32_t cmd[4];
      mecanumMix(p.vx, p.vy, p.omega, cmd);
      // Slew-limit toward target to cap simultaneous inrush current.
      int32_t maxStep = (int32_t)(CMD_SLEW * dt);
      if (maxStep < 1) maxStep = 1;
      for (int i = 0; i < 4; i++) {
        int32_t d = cmd[i] - curCmd[i];
        if (d >  maxStep) d =  maxStep;
        if (d < -maxStep) d = -maxStep;
        curCmd[i] += d;
      }
      pidStep(curCmd, dt);
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
    Serial.printf("seq=%lu vx=%d vy=%d w=%d | cmd=[%ld %ld %ld %ld] pwm=[%.0f %.0f %.0f %.0f] meas=[%.0f %.0f %.0f %.0f]%s\n",
                  (unsigned long)p.seq, p.vx, p.vy, p.omega,
                  (long)cmd[0], (long)cmd[1], (long)cmd[2], (long)cmd[3],
                  lastOutPwm[0], lastOutPwm[1], lastOutPwm[2], lastOutPwm[3],
                  lastMeasTps[0], lastMeasTps[1], lastMeasTps[2], lastMeasTps[3],
                  fresh ? "" : " (stale)");
    lastSeen = p.seq;
  }
}
