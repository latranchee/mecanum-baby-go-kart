#include <M5Unified.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"
#include "config_controller.h"  // pulls in curve.h (CurveCfg/CURVE) + tunables
#include "control_math.h"       // normalize(), clampI32(), crc8()

// Atom JoyStick: AtomS3 (ESP32-S3) + STM32 co-processor at I2C 0x59.
// I2C: SDA=GPIO38, SCL=GPIO39, 400kHz on Wire1 (matches original firmware).
// Reg map (Atom-JoyStick-Internal-FW):
//   0x00 [4B]: X1 lo,hi, Y1 lo,hi   (left stick, 12-bit LE)
//   0x20 [4B]: X2 lo,hi, Y2 lo,hi   (right stick, 12-bit LE)
//   0x70 [4B]: LeftBtn, RightBtn, LeftJoyBtn, RightJoyBtn (1B each, 1=released)

static const uint8_t JOY_ADDR    = 0x59;
// Reg map (matches user's physical sticks, validated by on-screen debug):
//   0x00 -> physical LEFT  stick
//   0x20 -> physical RIGHT stick
// Within each reg: bytes 0-1 = HORIZ axis (left/right tilt), bytes 2-3 = VERT axis (up/down tilt).
static const uint8_t REG_STICK_LEFT_PHYS  = 0x00;
static const uint8_t REG_STICK_RIGHT_PHYS = 0x20;
static const uint8_t REG_BTNS             = 0x70;

static const int I2C_SDA = 38;
static const int I2C_SCL = 39;

// "Left/Right" below = USER's physical sticks (not M5 reg labels — see above).
static uint16_t centerLHoriz = 2048, centerLVert = 2048;
static uint16_t centerRHoriz = 2048, centerRVert = 2048;

// DEADZONE_RAW / HALF_RANGE in config_controller.h. CurveCfg/CURVE in curve.h
// (via config_controller.h); applyCurve() in curve.h.

static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire1.beginTransmission(JOY_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  uint8_t got = Wire1.requestFrom(JOY_ADDR, n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire1.read();
  return true;
}

// I2C reg layout (per on-screen debug): bytes 0-1 = HORIZ axis, bytes 2-3 = VERT axis.
static bool readStick(uint8_t reg, uint16_t* horiz, uint16_t* vert) {
  uint8_t b[4];
  if (!readRegs(reg, b, 4)) return false;
  *horiz = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  *vert  = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
  return true;
}

static bool readButtons(uint8_t out[4]) {
  return readRegs(REG_BTNS, out, 4);
}

// normalize() now in include/control_math.h (host-testable). Thin wrapper binds
// the controller's tuning constants so call sites stay 2-arg.
static inline int16_t normalize(uint16_t raw, uint16_t center) {
  return normalize(raw, center, DEADZONE_RAW, HALF_RANGE);
}

static void calibrateCenter() {
  uint32_t sumLH = 0, sumLV = 0, sumRH = 0, sumRV = 0;
  const int N = 16;
  int ok = 0;
  for (int i = 0; i < N; i++) {
    uint16_t lh, lv, rh, rv;
    // PHYS_LEFT reg holds user's left stick; PHYS_RIGHT reg holds user's right stick.
    if (readStick(REG_STICK_LEFT_PHYS,  &lh, &lv) &&
        readStick(REG_STICK_RIGHT_PHYS, &rh, &rv)) {
      sumLH += lh; sumLV += lv; sumRH += rh; sumRV += rv;
      ok++;
    }
    delay(10);
  }
  if (ok > 0) {
    centerLHoriz = sumLH / ok;
    centerLVert  = sumLV / ok;
    centerRHoriz = sumRH / ok;
    centerRVert  = sumRV / ok;
  }
  Serial.printf("Centers: L=(h%u,v%u) R=(h%u,v%u) n=%d\n",
                centerLHoriz, centerLVert, centerRHoriz, centerRVert, ok);
}

// ---------------- ESP-NOW ----------------
static volatile uint32_t lastAckMs   = 0;
static volatile bool     lastAckOK   = false;
static bool              peerAdded   = false;

static void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  lastAckOK = (status == ESP_NOW_SEND_SUCCESS);
  if (lastAckOK) lastAckMs = millis();
}

static void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("Controller MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ROBOT_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) == ESP_OK) {
    peerAdded = true;
    Serial.println("Peer added");
  } else {
    Serial.println("add_peer FAILED");
  }
}

// ---------------- Display ----------------
// AtomS3 LCD is 128x128. Layout:
//   Header bar: "MECANUM"
//   Status:     "ONLINE" green / "OFFLINE" red (last-ack age)
//   Speed:      big number "  70%"
//   Footer:     seq counter
static uint8_t  speedPct      = 50;       // 10..100, step 10
static uint8_t  lastSpeedPct  = 0;
static bool     lastConnected = false;
static uint32_t lastDispMs    = 0;

static void drawHeader() {
  M5.Display.fillRect(0, 0, 128, 16, TFT_DARKGREY);
  M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
  M5.Display.setTextDatum(top_center);
  M5.Display.setTextSize(1);
  M5.Display.drawString("MECANUM", 64, 4);
}

static void drawStatus(bool connected) {
  uint16_t bg = connected ? TFT_DARKGREEN : TFT_RED;
  M5.Display.fillRect(0, 16, 128, 16, bg);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.drawString(connected ? "ONLINE" : "OFFLINE", 64, 24);
}

static void drawSpeedCompact(uint8_t pct) {
  M5.Display.fillRect(0, 34, 128, 10, TFT_BLACK);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.setTextDatum(top_left);
  M5.Display.setTextSize(1);
  char buf[16];
  snprintf(buf, sizeof(buf), "SPD %u%%", pct);
  M5.Display.drawString(buf, 4, 35);
}

// Physical-deflection labels (sign convention validated by working code):
//   vert: normalize > 0 => stick DOWN (raw above center)
//   horiz: normalize > 0 => stick RIGHT
static const char* vertLabel(int n)  { return (n > 0) ? "DN" : ((n < 0) ? "UP" : "--"); }
static const char* horizLabel(int n) { return (n > 0) ? "RT" : ((n < 0) ? "LT" : "--"); }

static void drawSticks(int16_t lVertN, int16_t lHorizN, int16_t rVertN, int16_t rHorizN) {
  M5.Display.fillRect(0, 46, 128, 82, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(top_left);
  M5.Display.setTextSize(2);
  char buf[16];
  snprintf(buf, sizeof(buf), "L V:%s%3d", vertLabel(lVertN),   abs(lVertN)/10);
  M5.Display.drawString(buf, 2, 48);
  snprintf(buf, sizeof(buf), "L H:%s%3d", horizLabel(lHorizN), abs(lHorizN)/10);
  M5.Display.drawString(buf, 2, 68);
  snprintf(buf, sizeof(buf), "R V:%s%3d", vertLabel(rVertN),   abs(rVertN)/10);
  M5.Display.drawString(buf, 2, 88);
  snprintf(buf, sizeof(buf), "R H:%s%3d", horizLabel(rHorizN), abs(rHorizN)/10);
  M5.Display.drawString(buf, 2, 108);
}

static void initDisplay() {
  M5.Display.setRotation(0);
  M5.Display.fillScreen(TFT_BLACK);
  drawHeader();
  drawStatus(false);
  drawSpeedCompact(speedPct);
  drawSticks(0, 0, 0, 0);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  delay(300);
  Serial.println("\nmecanum controller: Atom JoyStick -> ESP-NOW");

  initDisplay();

  Wire1.begin(I2C_SDA, I2C_SCL);
  Wire1.setClock(400000);
  delay(200);

  calibrateCenter();
  setupEspNow();
}

static uint32_t lastSendMs = 0;
static uint32_t lastLogMs  = 0;
static uint32_t seq        = 0;
static uint8_t  lastBtnMask = 0;

void loop() {
  uint32_t now = millis();

  if (now - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = now;

  uint16_t lHoriz, lVert, rHoriz, rVert;
  uint8_t  btns[4] = { 1, 1, 1, 1 };
  bool ok = readStick(REG_STICK_LEFT_PHYS,  &lHoriz, &lVert) &&
            readStick(REG_STICK_RIGHT_PHYS, &rHoriz, &rVert) &&
            readButtons(btns);
  if (!ok) return;

  // Buttons: 0 = pressed
  uint8_t btnMask = 0;
  if (btns[0] == 0) btnMask |= 0x01;  // LeftBtn  (yellow L, top)  = decrease speed
  if (btns[1] == 0) btnMask |= 0x02;  // RightBtn (yellow R, top)  = increase speed
  if (btns[2] == 0) btnMask |= 0x04;  // LeftJoyBtn (stick press)
  if (btns[3] == 0) btnMask |= 0x08;  // RightJoyBtn (stick press)

  // Edge-triggered speed adjust on top buttons
  uint8_t pressed = btnMask & ~lastBtnMask;
  if (pressed & 0x02) speedPct = (speedPct >= 100)         ? 100        : speedPct + SPEED_STEP;
  if (pressed & 0x01) speedPct = (speedPct <= SPEED_STEP)  ? SPEED_STEP : speedPct - SPEED_STEP;
  lastBtnMask = btnMask;

  // Normalize stick deflections to [-1000..+1000]. Names reflect actual physical axes.
  int16_t lHorizN = normalize(lHoriz, centerLHoriz);
  int16_t lVertN  = normalize(lVert,  centerLVert);
  int16_t rHorizN = normalize(rHoriz, centerRHoriz);
  int16_t rVertN  = normalize(rVert,  centerRVert);

  // Stick -> robot mapping.
  //   LEFT  stick: VERT (UP=fwd)    -> vx primary
  //                HORIZ (RIGHT=R)  -> vy (strafe right)
  //   RIGHT stick: VERT (UP=fwd)    -> vx add (both stick verts sum)
  //                HORIZ (RIGHT=CW) -> omega (CW = negative; protocol: omega>0 CCW)
  // normalize() returns +DOWN / +RIGHT (see lines 193-195), so UP requires negation.
  int16_t vy_raw    =  lHorizN;
  int16_t omega_raw = -rHorizN;
  // vx: sum both stick verts, clamp to normalized range (opposing pushes cancel).
  int16_t vx_raw    = (int16_t)constrain((int32_t)(-lVertN) + (int32_t)(-rVertN), -1000, 1000);

  // Apply S-curve then scale by speedPct (top-end limit).
  float scale = (float)speedPct / 100.0f;
  int16_t vx    = (int16_t)(applyCurve(vx_raw,    CURVE) * scale * 1000.0f);
  int16_t vy    = (int16_t)(applyCurve(vy_raw,    CURVE) * scale * 1000.0f);
  int16_t omega = (int16_t)(applyCurve(omega_raw, CURVE) * scale * 1000.0f);

  // E-stop: both joystick clicks held
  uint8_t flags = 0;
  if ((btnMask & 0x0C) == 0x0C) flags |= 0x01;

  CtrlPacket pkt = { ++seq, vx, vy, omega, btnMask, flags };
  if (peerAdded) esp_now_send(ROBOT_MAC, (uint8_t*)&pkt, sizeof(pkt));

  // Display refresh @ ~10Hz: status/speed only on change, sticks every tick.
  if (now - lastDispMs >= 100) {
    lastDispMs = now;
    bool connected = (now - lastAckMs) < 500;
    if (connected != lastConnected) {
      drawStatus(connected);
      lastConnected = connected;
    }
    if (speedPct != lastSpeedPct) {
      drawSpeedCompact(speedPct);
      lastSpeedPct = speedPct;
    }
    drawSticks(lVertN, lHorizN, rVertN, rHorizN);
  }

  // Serial log @ 5Hz
  if (now - lastLogMs >= 200) {
    lastLogMs = now;
    Serial.printf("seq=%lu spd=%u%% btn=%02X conn=%d L=(h%u,v%u) R=(h%u,v%u) -> vx=%d vy=%d w=%d\n",
                  (unsigned long)seq, speedPct, btnMask,
                  (now - lastAckMs) < 500 ? 1 : 0,
                  lHoriz, lVert, rHoriz, rVert, vx, vy, omega);
  }
}
