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

static const Motor motors[4] = {
  { 25, 33, 27, 39, 36 },  // M1 FL
  { 26, 32, 14, 35, 34 },  // M2 FR
  { 18,  5, 19, 16,  4 },  // M3 RL
  { 21, 22, 23, 13, 17 },  // M4 RR
};

static const int PWM_FREQ = 25000;
static const int PWM_RES  = 10;
static const int PWM_MAX  = (1 << PWM_RES) - 1;
static const int DEADBAND = 0;  // disabled: PID feed-forward handles low-speed PWM directly

static volatile long encCount[4] = { 0, 0, 0, 0 };
static const int8_t encSign[4]   = { -1, -1, +1, +1 };

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
static const float Ki      = 0.0f;             // disabled — windup pushed PWM to saturation
static const float I_MAX   = (float)PWM_MAX;

struct PidState {
  float  integral;
  long   lastCount;
};
static PidState pid[4] = {};

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
      // Hard stop: drain integral so motor doesn't creep.
      pid[i].integral = 0.0f;
      motorWrite(i, 0);
      continue;
    }

    float err = targetTps - measuredTps;
    pid[i].integral += err * dt * Ki;
    if (pid[i].integral >  I_MAX) pid[i].integral =  I_MAX;
    if (pid[i].integral < -I_MAX) pid[i].integral = -I_MAX;

    float out = Kff * targetTps + Kp * err + pid[i].integral;
    if (out >  PWM_MAX) out =  PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;

    motorWrite(i, (int16_t)out);
    lastTargetTps[i] = targetTps;
    lastMeasTps[i]   = measuredTps;
    lastOutPwm[i]    = out;
  }
}

// ---------------- ESP-NOW receive ----------------
static volatile uint32_t lastPacketMs = 0;
static CtrlPacket lastPacket = { 0, 0, 0, 0, 0, 0 };
static portMUX_TYPE pktMux = portMUX_INITIALIZER_UNLOCKED;

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(CtrlPacket)) return;
  portENTER_CRITICAL(&pktMux);
  memcpy(&lastPacket, data, sizeof(CtrlPacket));
  lastPacketMs = millis();
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

  if (now - lastDriveMs >= 10) {
    float dt = (now - lastDriveMs) / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;
    lastDriveMs = now;

    CtrlPacket p;
    uint32_t age;
    portENTER_CRITICAL(&pktMux);
    p   = lastPacket;
    age = now - lastPacketMs;
    portEXIT_CRITICAL(&pktMux);

    if (age > 500 || (p.flags & 0x01)) {
      if (!wasStopped) { motorStopAll(); pidReset(); wasStopped = true; }
    } else {
      int32_t cmd[4];
      mecanumMix(p.vx, p.vy, p.omega, cmd);
      pidStep(cmd, dt);
      wasStopped = false;
    }
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
    Serial.printf("seq=%lu age=%lums vx=%d vy=%d w=%d cmd=[%ld %ld %ld %ld] M1: tgt=%.0f meas=%.0f pwm=%.0f%s\n",
                  (unsigned long)p.seq, (unsigned long)age, p.vx, p.vy, p.omega,
                  (long)cmd[0], (long)cmd[1], (long)cmd[2], (long)cmd[3],
                  lastTargetTps[0], lastMeasTps[0], lastOutPwm[0],
                  fresh ? "" : " (stale)");
    lastSeen = p.seq;
  }
}
