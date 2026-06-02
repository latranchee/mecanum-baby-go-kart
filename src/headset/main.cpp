// Head-tilt controller for the mecanum cart.
// Board: Waveshare ESP32-S3-LCD-1.69 (ST7789V2 240x280 IPS + QMI8658 6-axis IMU).
//
// Acts as a drop-in ESP-NOW transmitter: builds the same CtrlPacket the Atom
// JoyStick sends and unicasts it to ROBOT_MAC on ESPNOW_CHANNEL at 50Hz. The
// robot firmware is unchanged (its onRecv accepts any sender on the channel).
//
// Head mapping (board mounted so its +X points out the nose, +Z up):
//   pitch (nod down/up)        -> vx     (forward / back)
//   roll  (ear-to-shoulder)    -> vy     (strafe right / left)
//   yaw   (look left / right)  -> omega  (turn CCW / CW)
//
// pitch & roll come from the accelerometer (gravity reference) so they never
// drift. yaw is integrated gyro-Z (no magnetometer on QMI8658) so it drifts
// slowly -> press BOOT to recenter (zeroes yaw and captures the neutral pose).
//
// SAFETY: always armed (per build choice). Test wheels-up first. Recenter with
// a still, level head. SPEED_CAP limits top speed.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include "SensorQMI8658.hpp"
#include <Arduino_GFX_Library.h>
#include "protocol.h"

// RGB565 palette (Arduino_GFX uses RGB565_* names; define plain ones we use).
#ifndef BLACK
#define BLACK    0x0000
#define WHITE    0xFFFF
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define DARKGREY 0x7BEF
#endif

// ---------------- Pin map (Waveshare ESP32-S3-LCD-1.69) ----------------
static const int LCD_SCK  = 6;
static const int LCD_MOSI = 7;
static const int LCD_DC   = 4;
static const int LCD_CS   = 5;
static const int LCD_RST  = 8;
static const int LCD_BL   = 15;

static const int IMU_SDA  = 11;
static const int IMU_SCL  = 10;

static const int PIN_BOOT = 0;   // BOOT button (active LOW) = recenter

// ---------------- Tunables ----------------
static const float FULL_TILT_DEG = 30.0f;  // pitch/roll angle that = full speed
static const float FULL_YAW_DEG  = 60.0f;  // yaw angle from center that = full turn
static const float SPEED_CAP     = 0.60f;  // top-speed limit (0..1). Raise once trusted.

// Sign flips: set to -1 if an axis drives the wrong way after a bench test.
static const float INV_PITCH = -1.0f;  // nod forward  => +vx (forward)
static const float INV_ROLL  = +1.0f;  // tilt right   => +vy (strafe right)
static const float INV_YAW   = +1.0f;  // turn head left (CCW) => +omega (CCW)

static const float ALPHA = 0.98f;          // complementary filter (gyro weight)
static const uint32_t SEND_PERIOD_MS = 20; // 50Hz (robot watchdog = 500ms)

// ---------------- Stick response curve (copied from controller) ----------------
struct CurveCfg { int kind; float max; float deadzone; float k; float mid; float p; };
static const CurveCfg CURVE = { 3, 1.0f, 0.05f, 6.0f, 0.59f, 2.0f };  // logistic

static float applyCurve(float xNorm) {  // xNorm in [-1..1] -> shaped [-1..1]
  if (xNorm == 0.0f) return 0.0f;
  float sign = (xNorm < 0) ? -1.0f : 1.0f;
  float x = fabsf(xNorm);
  if (x <= CURVE.deadzone) return 0.0f;
  float t = (x - CURVE.deadzone) / (1.0f - CURVE.deadzone);
  if (t > 1.0f) t = 1.0f;
  float y;
  switch (CURVE.kind) {
    case 0: y = t; break;
    case 1: y = t * t * (3.0f - 2.0f * t); break;
    case 2: y = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); break;
    case 3: {
      float s0 = 1.0f / (1.0f + expf(-CURVE.k * (0.0f - CURVE.mid)));
      float s1 = 1.0f / (1.0f + expf(-CURVE.k * (1.0f - CURVE.mid)));
      float st = 1.0f / (1.0f + expf(-CURVE.k * (t - CURVE.mid)));
      y = (st - s0) / (s1 - s0);
      break;
    }
    case 4: y = powf(t, CURVE.p); break;
    default: y = t;
  }
  return sign * y * CURVE.max;
}

// ---------------- Display ----------------
static Arduino_DataBus* bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
// 240x280 panel sits in the ST7789 RAM with a 20px row offset.
static Arduino_GFX* gfx = new Arduino_ST7789(bus, LCD_RST, 0 /*rotation*/, true /*IPS*/,
                                             240, 280, 0, 20, 0, 20);

// ---------------- IMU ----------------
static SensorQMI8658 qmi;
static float pitch = 0, roll = 0, yaw = 0;   // filtered, degrees
static float pitchTrim = 0, rollTrim = 0;    // neutral pose captured at recenter
static float gyroBiasZ = 0;                  // dps, removed before integrating yaw

// ---------------- ESP-NOW ----------------
static volatile uint32_t lastAckMs = 0;
static bool peerAdded = false;

static void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS) lastAckMs = millis();
}

static void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.print("Headset MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init FAILED"); return; }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ROBOT_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) == ESP_OK) { peerAdded = true; Serial.println("Peer added"); }
  else Serial.println("add_peer FAILED");
}

// ---------------- IMU helpers ----------------
// Calibrate gyro Z bias + capture neutral pitch/roll. Head must be still & level.
static void recenter() {
  const int N = 100;
  float sumGz = 0, sumP = 0, sumR = 0;
  IMUdata acc, gyr;
  for (int i = 0; i < N; i++) {
    if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)
                           && qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
      sumGz += gyr.z;
      sumP  += atan2f(acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
      sumR  += atan2f(acc.y, acc.z) * RAD_TO_DEG;
    }
    delay(5);
  }
  gyroBiasZ = sumGz / N;
  pitchTrim = sumP / N;
  rollTrim  = sumR / N;
  pitch = pitchTrim; roll = rollTrim; yaw = 0;
  Serial.printf("Recenter: pitchTrim=%.1f rollTrim=%.1f gyroBiasZ=%.3f\n",
                pitchTrim, rollTrim, gyroBiasZ);
}

static void setupImu() {
  Wire.begin(IMU_SDA, IMU_SCL, 400000);
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IMU_SDA, IMU_SCL)) {
    Serial.println("QMI8658 not found! check wiring / I2C addr");
    gfx->setCursor(4, 120);
    gfx->setTextColor(RED);
    gfx->print("IMU FAIL");
  }
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_250Hz);
  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_224_2Hz);
  qmi.enableAccelerometer();
  qmi.enableGyroscope();
}

// ---------------- HUD ----------------
static void drawStaticHud() {
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(8, 6);
  gfx->print("HEAD CTRL");
  gfx->setTextSize(1);
  gfx->setTextColor(DARKGREY);
  gfx->setCursor(8, 262);
  gfx->print("BOOT = recenter");
}

// signed bar centered at x..x+w, value -1..1, with a label
static void drawBar(int y, const char* label, float v, uint16_t col) {
  const int x = 70, w = 160, h = 22;
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(6, y + 3);
  gfx->print(label);
  gfx->drawRect(x, y, w, h, DARKGREY);
  gfx->fillRect(x + 1, y + 1, w - 2, h - 2, BLACK);
  int mid = x + w / 2;
  gfx->drawFastVLine(mid, y, h, DARKGREY);
  if (v > 1) v = 1; if (v < -1) v = -1;
  int len = (int)(v * (w / 2 - 2));
  if (len >= 0) gfx->fillRect(mid, y + 2, len, h - 4, col);
  else          gfx->fillRect(mid + len, y + 2, -len, h - 4, col);
}

static uint32_t lastHudMs = 0;
static void drawHud(int16_t vx, int16_t vy, int16_t omega) {
  bool online = (millis() - lastAckMs) < 500;
  gfx->fillRect(8, 34, 224, 18, BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(online ? GREEN : RED, BLACK);
  gfx->setCursor(8, 34);
  gfx->print(online ? "LINK OK " : "NO LINK ");

  drawBar(70,  "FW", vx    / 1000.0f, CYAN);
  drawBar(105, "ST", vy    / 1000.0f, MAGENTA);
  drawBar(140, "TN", omega / 1000.0f, YELLOW);

  gfx->fillRect(0, 175, 240, 60, BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(8, 178);  gfx->printf("P:%+5.1f", pitch - pitchTrim);
  gfx->setCursor(8, 200);  gfx->printf("R:%+5.1f", roll  - rollTrim);
  gfx->setCursor(128, 178); gfx->printf("Y:%+5.1f", yaw);
}

// ---------------- main ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nmecanum headset: QMI8658 tilt -> ESP-NOW");

  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  gfx->begin();
  drawStaticHud();

  setupImu();
  setupEspNow();

  delay(200);
  Serial.println("Hold head still & level for recenter...");
  recenter();
}

static uint32_t lastSendMs = 0;
static uint32_t lastStepMs = 0;
static uint32_t lastLogMs  = 0;
static uint32_t seq        = 0;
static bool     lastBoot   = true;

void loop() {
  uint32_t now = millis();

  // BOOT button (active LOW) edge -> recenter
  bool boot = digitalRead(PIN_BOOT);
  if (!boot && lastBoot) recenter();
  lastBoot = boot;

  // IMU fusion step
  IMUdata acc, gyr;
  if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)
                         && qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
    float dt = (lastStepMs == 0) ? 0.004f : (now - lastStepMs) / 1000.0f;
    lastStepMs = now;
    if (dt <= 0) dt = 0.004f;

    float pitchAcc = atan2f(acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
    float rollAcc  = atan2f(acc.y, acc.z) * RAD_TO_DEG;
    // gyr in dps. gyr.y ~ pitch rate, gyr.x ~ roll rate (mounting-dependent;
    // accel term corrects any sign error over time).
    pitch = ALPHA * (pitch + gyr.y * dt) + (1 - ALPHA) * pitchAcc;
    roll  = ALPHA * (roll  + gyr.x * dt) + (1 - ALPHA) * rollAcc;
    yaw  += (gyr.z - gyroBiasZ) * dt;
  }

  if (now - lastSendMs < SEND_PERIOD_MS) return;
  lastSendMs = now;

  // Pose -> normalized -> S-curve -> scaled command
  float fwd = INV_PITCH * (pitch - pitchTrim) / FULL_TILT_DEG;
  float str = INV_ROLL  * (roll  - rollTrim)  / FULL_TILT_DEG;
  float trn = INV_YAW   *  yaw                / FULL_YAW_DEG;

  int16_t vx    = (int16_t)(applyCurve(fwd) * SPEED_CAP * 1000.0f);
  int16_t vy    = (int16_t)(applyCurve(str) * SPEED_CAP * 1000.0f);
  int16_t omega = (int16_t)(applyCurve(trn) * SPEED_CAP * 1000.0f);

  CtrlPacket pkt = { ++seq, vx, vy, omega, 0, 0 };
  if (peerAdded) esp_now_send(ROBOT_MAC, (uint8_t*)&pkt, sizeof(pkt));

  if (now - lastHudMs >= 100) { lastHudMs = now; drawHud(vx, vy, omega); }

  if (now - lastLogMs >= 250) {
    lastLogMs = now;
    Serial.printf("seq=%lu P=%+.1f R=%+.1f Y=%+.1f -> vx=%d vy=%d w=%d conn=%d\n",
                  (unsigned long)seq, pitch - pitchTrim, roll - rollTrim, yaw,
                  vx, vy, omega, (now - lastAckMs) < 500 ? 1 : 0);
  }
}
