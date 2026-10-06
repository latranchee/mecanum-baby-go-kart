#include <M5Unified.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>            // esp_read_mac()
#include "protocol.h"
#include "config_controller.h"  // pulls in curve.h (CurveCfg/CURVE) + tunables
#include "control_math.h"       // normalize(), clampI32(), crc8()
#include "controller_logic.h"   // stick calibration, arming, mode-click gesture
#include "safety.h"             // linkAgeMs()

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
// Axis order used by the calibration arrays: L horiz, L vert, R horiz, R vert.
enum Axis : uint8_t { AX_LH = 0, AX_LV = 1, AX_RH = 2, AX_RV = 3 };
static uint16_t center[4] = { CENTER_REST[0], CENTER_REST[1], CENTER_REST[2], CENTER_REST[3] };

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

static void beginJoyBus() {
  Wire1.begin(I2C_SDA, I2C_SCL);
  Wire1.setClock(400000);
}

// normalize() now in include/control_math.h (host-testable). Thin wrapper binds
// the controller's tuning constants so call sites stay 2-arg.
static inline int16_t normalize(uint16_t raw, uint16_t c) {
  return normalize(raw, c, DEADZONE_RAW, HALF_RANGE);
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

  // Factory STA MAC from eFuse (WiFi.macAddress() can read all zeros this early on
  // Arduino core 3.x). secrets.h CONTROLLER_MAC is copied from this line.
  uint8_t mac[6] = {};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }
  esp_now_register_send_cb(onSent);

#if ESPNOW_ENCRYPT
  esp_now_set_pmk((const uint8_t*)ESPNOW_PMK);  // must precede add_peer (#3)
#endif

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ROBOT_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
#if ESPNOW_ENCRYPT
  peer.encrypt = true;
  memcpy(peer.lmk, ESPNOW_LMK, 16);
#else
  peer.encrypt = false;
#endif
  if (esp_now_add_peer(&peer) == ESP_OK) {
    peerAdded = true;
    Serial.println("Peer added");
  } else {
    Serial.println("add_peer FAILED");
  }
}

static uint32_t seq = 0;

static void sendPacket(int16_t vx, int16_t vy, int16_t omega, uint8_t buttons, uint8_t flags) {
  CtrlPacket pkt = { ++seq, vx, vy, omega, buttons, flags, 0 };
  pkt.crc = crc8((const uint8_t*)&pkt, offsetof(CtrlPacket, crc));  // integrity (#4)
  if (peerAdded) esp_now_send(ROBOT_MAC, (uint8_t*)&pkt, sizeof(pkt));
}

// ---------------- Feature-mode presets ----------------
// Left joystick-click cycles this ordered list (on release, see controller_logic.h
// modeClickStep). Each entry = display name + the DISABLE bits sent in
// CtrlPacket.flags (protocol.h). FULL = nothing disabled = every robot feature ON
// (legacy behaviour). The robot mirrors these bits into its runtime toggles each
// packet, so the controller is authoritative in the field.
struct ModePreset { const char* name; uint8_t disableBits; };
static const ModePreset MODE_PRESETS[] = {
  { "FULL",    0 },
  { "NO GOV",  CTRL_FLAG_GOV_OFF },
  { "NO BODY", CTRL_FLAG_BODY_OFF },
  { "OPEN LP", CTRL_FLAG_CL_OFF },
  { "RAW",     CTRL_FLAG_GOV_OFF | CTRL_FLAG_CL_OFF | CTRL_FLAG_BODY_OFF },
};
static const uint8_t NMODES      = sizeof(MODE_PRESETS) / sizeof(MODE_PRESETS[0]);
static uint8_t       modeIdx     = 0;
static uint8_t       lastModeIdx = 0xFF;   // != any valid idx -> force first draw
static ModeClick     modeClick   = {};

// ---------------- Input state ----------------
static StickCal cal         = {};
static bool     calibrated  = false;  // trusted stick centre found (controller_logic.h)
static bool     armed       = false;  // drive commands allowed (controller_logic.h armStep)
static bool     estopHeld   = false;  // both stick clicks down on the last good read
static uint16_t joyFails    = 0;      // consecutive failed I2C reads
static uint32_t lastReinitMs = 0;

// ---------------- Display ----------------
// AtomS3 LCD is 128x128. Layout:
//   Header bar: "MECANUM"
//   Status bar: one state, highest priority first (StatusView below)
//   SPD row:    speed % left, mode preset right
//   Sticks:     four rows, redrawn on change
static uint8_t  speedPct      = 50;       // 10..100, step 10
static uint8_t  lastSpeedPct  = 0;
static uint32_t lastDispMs    = 0;

// What the status bar shows. Everything that stops the cart has its own word, so
// the screen never says ONLINE while the controller is not driving.
enum StatusView : uint8_t { SV_NONE, SV_NO_RADIO, SV_JOY_ERR, SV_ESTOP, SV_CENTER, SV_ONLINE, SV_OFFLINE };
static StatusView lastView = SV_NONE;

static StatusView currentView(uint32_t now) {
  if (!peerAdded)                return SV_NO_RADIO;   // radio dead: nothing is sent
  if (joyFails >= JOY_FAIL_TRIP) return SV_JOY_ERR;    // sticks unreadable: e-stop sent
  if (estopHeld)                 return SV_ESTOP;
  if (!calibrated || !armed)     return SV_CENTER;     // release / centre the sticks
  // Signed age: onSent stamps lastAckMs on core 0 and can land after `now` was
  // read here; an unsigned difference wrapped and flashed OFFLINE for a refresh.
  return linkAgeMs(now, lastAckMs) < 500 ? SV_ONLINE : SV_OFFLINE;
}

static void drawHeader() {
  M5.Display.fillRect(0, 0, 128, 16, TFT_DARKGREY);
  M5.Display.setTextColor(TFT_WHITE, TFT_DARKGREY);
  M5.Display.setTextDatum(top_center);
  M5.Display.setTextSize(1);
  M5.Display.drawString("MECANUM", 64, 4);
}

static void drawStatus(StatusView v) {
  const char* text = "";
  uint16_t bg = TFT_RED, fg = TFT_WHITE;
  switch (v) {
    case SV_NO_RADIO: text = "NO RADIO"; break;
    case SV_JOY_ERR:  text = "JOY ERR";  break;
    case SV_ESTOP:    text = "E-STOP";   break;
    case SV_CENTER:   text = "CENTER";   bg = TFT_ORANGE; fg = TFT_BLACK; break;
    case SV_ONLINE:   text = "ONLINE";   bg = TFT_DARKGREEN; break;
    case SV_OFFLINE:  text = "OFFLINE";  break;
    default: break;
  }
  M5.Display.fillRect(0, 16, 128, 16, bg);
  M5.Display.setTextColor(fg, bg);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(2);
  M5.Display.drawString(text, 64, 24);
}

static void drawSpeedCompact(uint8_t pct) {
  M5.Display.fillRect(0, 34, 60, 10, TFT_BLACK);   // left half — drawMode owns right
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.setTextDatum(top_left);
  M5.Display.setTextSize(1);
  char buf[16];
  snprintf(buf, sizeof(buf), "SPD %u%%", pct);
  M5.Display.drawString(buf, 4, 35);
}

// Active feature-mode preset name, right side of the SPD row.
static void drawMode(uint8_t idx) {
  M5.Display.fillRect(60, 34, 68, 10, TFT_BLACK);  // right half of the SPD row
  M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Display.setTextDatum(top_right);
  M5.Display.setTextSize(1);
  M5.Display.drawString(MODE_PRESETS[idx].name, 124, 35);
}

// Physical-deflection labels (sign convention validated by working code):
//   vert: normalize > 0 => stick DOWN (raw above center)
//   horiz: normalize > 0 => stick RIGHT
static const char* vertLabel(int n)  { return (n > 0) ? "DN" : ((n < 0) ? "UP" : "--"); }
static const char* horizLabel(int n) { return (n > 0) ? "RT" : ((n < 0) ? "LT" : "--"); }

// Stick readout: four fixed-width rows ("L V:UP 42"). A row is redrawn only when
// its text changed, and with an opaque background, so it overwrites itself in
// place. No panel clear: that was a 128x82 fill plus four rows of text over SPI
// every 100 ms whether the sticks moved or not, and it flickered.
static void drawSticks(int16_t lVertN, int16_t lHorizN, int16_t rVertN, int16_t rHorizN) {
  static char shown[4][16];   // text on screen per row; "" until first drawn
  char rows[4][16];
  snprintf(rows[0], sizeof(rows[0]), "L V:%s%3d", vertLabel(lVertN),   abs(lVertN)/10);
  snprintf(rows[1], sizeof(rows[1]), "L H:%s%3d", horizLabel(lHorizN), abs(lHorizN)/10);
  snprintf(rows[2], sizeof(rows[2]), "R V:%s%3d", vertLabel(rVertN),   abs(rVertN)/10);
  snprintf(rows[3], sizeof(rows[3]), "R H:%s%3d", horizLabel(rHorizN), abs(rHorizN)/10);

  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(top_left);
  M5.Display.setTextSize(2);
  for (uint8_t r = 0; r < 4; r++) {
    if (strcmp(rows[r], shown[r]) == 0) continue;
    M5.Display.drawString(rows[r], 2, 48 + 20 * r);
    strcpy(shown[r], rows[r]);
  }
}

// Status/speed/mode on change, sticks every call. Runs on every send tick at
// ~10 Hz — including ticks where the joystick read failed, so the status bar
// always tells the truth.
static void refreshDisplay(uint32_t now, const int16_t* sticks /* lV,lH,rV,rH or null */) {
  if (now - lastDispMs < 100) return;
  lastDispMs = now;
  StatusView v = currentView(now);
  if (v != lastView) {
    drawStatus(v);
    lastView = v;
  }
  if (speedPct != lastSpeedPct) {
    drawSpeedCompact(speedPct);
    lastSpeedPct = speedPct;
  }
  if (modeIdx != lastModeIdx) {
    drawMode(modeIdx);
    lastModeIdx = modeIdx;
  }
  if (sticks) drawSticks(sticks[0], sticks[1], sticks[2], sticks[3]);
}

static void initDisplay() {
  M5.Display.setRotation(0);
  M5.Display.fillScreen(TFT_BLACK);
  drawHeader();
  drawSpeedCompact(speedPct);
  drawMode(modeIdx);
  drawSticks(0, 0, 0, 0);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  delay(300);
  Serial.println("\nmecanum controller: Atom JoyStick -> ESP-NOW");

  initDisplay();

  beginJoyBus();
  delay(200);

  // Stick centres are calibrated in loop() from readings taken while the sticks
  // rest (controller_logic.h stickCalAdd); nothing is sent until then.
  setupEspNow();
  refreshDisplay(millis(), nullptr);
}

static uint32_t lastSendMs  = 0;
static uint32_t lastLogMs   = 0;
static uint8_t  lastBtnMask = 0;

void loop() {
  uint32_t now = millis();

  if (now - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = now;

  uint16_t raw[4];   // AX_LH, AX_LV, AX_RH, AX_RV
  uint8_t  btns[4] = { 1, 1, 1, 1 };
  bool ok = readStick(REG_STICK_LEFT_PHYS,  &raw[AX_LH], &raw[AX_LV]) &&
            readStick(REG_STICK_RIGHT_PHYS, &raw[AX_RH], &raw[AX_RV]) &&
            readButtons(btns);

  // Joystick bus fault. A few failed reads are skipped (the robot holds the last
  // command, inside its 500 ms watchdog). Past JOY_FAIL_TRIP (~100 ms): disarm,
  // send e-stop frames so the robot stops now rather than at its watchdog, show
  // JOY ERR, and re-initialize the bus every JOY_REINIT_MS until it answers.
  // Recovery requires re-arming at neutral sticks.
  if (!ok) {
    if (joyFails < 0xFFFF) joyFails++;
    if (joyFails >= JOY_FAIL_TRIP) {
      if (joyFails == JOY_FAIL_TRIP) Serial.println("JOY ERR: joystick I2C not answering");
      armed     = false;
      estopHeld = false;
      sendPacket(0, 0, 0, 0, ctrlFlagsFromPreset(MODE_PRESETS[modeIdx].disableBits, true));
      if (now - lastReinitMs >= JOY_REINIT_MS) {
        lastReinitMs = now;
        Wire1.end();
        beginJoyBus();
      }
    }
    refreshDisplay(now, nullptr);
    return;
  }
  if (joyFails >= JOY_FAIL_TRIP) Serial.println("JOY ERR cleared");
  joyFails = 0;

  // Stick centre: only from a set of readings taken at rest, within CENTER_TOL of
  // this unit's measured rest (config_controller.h CENTER_REST). Until then
  // nothing is sent (the robot is stopped by its own watchdog).
  if (!calibrated) {
    static uint16_t rejects = 0;
    int r = stickCalAdd(&cal, raw, CAL_SAMPLES, CENTER_REST, CENTER_TOL, CENTER_SPREAD, center);
    if (r > 0) {
      calibrated = true;
      Serial.printf("Centers: L=(h%u,v%u) R=(h%u,v%u)\n",
                    center[AX_LH], center[AX_LV], center[AX_RH], center[AX_RV]);
    } else if (r < 0) {
      Serial.printf("calibration rejected (stick held or moving): L=(h%u,v%u) R=(h%u,v%u)\n",
                    raw[AX_LH], raw[AX_LV], raw[AX_RH], raw[AX_RV]);
      // ~5 s of rejections: if the sticks ARE released, the unit's rest has moved
      // off CENTER_REST (or this is a different joystick). Say so once.
      if (++rejects == 16)
        Serial.printf("hint: if the sticks are released, update CENTER_REST in "
                      "config_controller.h to the values above (now %u/%u/%u/%u, tol %u)\n",
                      CENTER_REST[0], CENTER_REST[1], CENTER_REST[2], CENTER_REST[3], CENTER_TOL);
    }
    refreshDisplay(now, nullptr);
    return;
  }

  // Buttons: 0 = pressed
  uint8_t btnMask = 0;
  if (btns[0] == 0) btnMask |= BTN_LEFT;   // yellow L (top) = decrease speed
  if (btns[1] == 0) btnMask |= BTN_RIGHT;  // yellow R (top) = increase speed
  if (btns[2] == 0) btnMask |= BTN_LJOY;   // left stick press
  if (btns[3] == 0) btnMask |= BTN_RJOY;   // right stick press

  // Edge-triggered speed adjust on top buttons
  uint8_t pressed = btnMask & ~lastBtnMask;
  if (pressed & BTN_RIGHT) speedPct = (speedPct >= 100)        ? 100        : speedPct + SPEED_STEP;
  if (pressed & BTN_LEFT)  speedPct = (speedPct <= SPEED_STEP) ? SPEED_STEP : speedPct - SPEED_STEP;
  // Left stick click cycles the feature mode on RELEASE, unless the right click
  // joined it (that is the e-stop gesture) — controller_logic.h modeClickStep.
  if (modeClickStep(&modeClick, btnMask, lastBtnMask)) modeIdx = (modeIdx + 1) % NMODES;
  lastBtnMask = btnMask;
  estopHeld = (btnMask & BTN_ESTOP) == BTN_ESTOP;

  // Disarmed (and not e-stopping): keep calibrating in the background and move
  // each axis centre only toward its measured rest (controller_logic.h centerImprove), so
  // a centre caught off a lightly held stick can't leave the controller stuck on
  // CENTER, and a stick held now can't become the centre.
  if (!armed && !estopHeld) {
    uint16_t cand[4];
    if (stickCalAdd(&cal, raw, CAL_SAMPLES, CENTER_REST, CENTER_TOL, CENTER_SPREAD, cand) > 0 &&
        centerImprove(center, cand, CENTER_REST)) {
      Serial.printf("Centers refined: L=(h%u,v%u) R=(h%u,v%u)\n",
                    center[AX_LH], center[AX_LV], center[AX_RH], center[AX_RV]);
    }
  } else {
    cal.n = 0;   // restart cleanly next time it disarms
  }

  // Normalize stick deflections to [-1000..+1000]. Names reflect actual physical axes.
  int16_t lHorizN = normalize(raw[AX_LH], center[AX_LH]);
  int16_t lVertN  = normalize(raw[AX_LV], center[AX_LV]);
  int16_t rHorizN = normalize(raw[AX_RH], center[AX_RH]);
  int16_t rVertN  = normalize(raw[AX_RV], center[AX_RV]);

  // Arming: the e-stop disarms; re-arm only once it is released AND every axis
  // is neutral. Disarmed = e-stop flag on every frame (latched e-stop), so letting
  // go of the clicks with a stick still pushed does not launch the cart.
  bool wasArmed = armed;
  armed = armStep(armed, estopHeld, sticksNeutral(lHorizN, lVertN, rHorizN, rVertN));
  if (armed != wasArmed) Serial.println(armed ? "ARMED" : "DISARMED (center sticks to re-arm)");

  // Stick -> robot mapping.
  //   LEFT  stick: VERT (UP=fwd)    -> vx primary
  //                HORIZ (RIGHT=R)  -> vy (strafe right)
  //   RIGHT stick: VERT (UP=fwd)    -> vx add (both stick verts sum)
  //                HORIZ (RIGHT=CW) -> omega (CW = negative; protocol: omega>0 CCW)
  // normalize() returns +DOWN / +RIGHT (see vertLabel/horizLabel), so UP requires
  // negation. Per-axis inversion via INVERT_* toggles in config_controller.h.
  const int sVx = INVERT_VX ? -1 : 1, sVy = INVERT_VY ? -1 : 1, sW = INVERT_OMEGA ? -1 : 1;
  int16_t vy_raw    = (int16_t)(sVy *  lHorizN);
  int16_t omega_raw = (int16_t)(sW  * -rHorizN);
  // vx: left stick vert always; right stick vert too unless decoupled (BUG-011).
  // Clamp to normalized range (opposing pushes cancel).
  int32_t vxSum = (int32_t)(-lVertN);
#if VX_FROM_BOTH_STICKS
  vxSum += (int32_t)(-rVertN);
#endif
  int16_t vx_raw    = (int16_t)constrain((int32_t)sVx * vxSum, -1000, 1000);

  // Apply response curve then scale by speedPct (top-end limit). omega uses the
  // gentler CURVE_OMEGA so small turns are responsive and additive (BUG-010/011).
  float scale = (float)speedPct / 100.0f;
  int16_t vx    = (int16_t)(applyCurve(vx_raw,    CURVE)       * scale * 1000.0f);
  int16_t vy    = (int16_t)(applyCurve(vy_raw,    CURVE)       * scale * 1000.0f);
  int16_t omega = (int16_t)(applyCurve(omega_raw, CURVE_OMEGA) * scale * 1000.0f);
  if (!armed) vx = vy = omega = 0;

  // Flags: e-stop (disarmed) ORed with the active preset's feature DISABLE bits.
  // flags==0 (FULL, armed) = all robot features ON.
  uint8_t flags = ctrlFlagsFromPreset(MODE_PRESETS[modeIdx].disableBits, !armed);
  sendPacket(vx, vy, omega, btnMask, flags);

  const int16_t sticks[4] = { lVertN, lHorizN, rVertN, rHorizN };
  refreshDisplay(now, sticks);

  // Serial log @ 5Hz
  if (now - lastLogMs >= 200) {
    lastLogMs = now;
    Serial.printf("seq=%lu spd=%u%% mode=%s btn=%02X armed=%d conn=%d L=(h%u,v%u) R=(h%u,v%u) -> vx=%d vy=%d w=%d\n",
                  (unsigned long)seq, speedPct, MODE_PRESETS[modeIdx].name, btnMask, armed ? 1 : 0,
                  linkAgeMs(now, lastAckMs) < 500 ? 1 : 0,
                  raw[AX_LH], raw[AX_LV], raw[AX_RH], raw[AX_RV], vx, vy, omega);
  }
}
