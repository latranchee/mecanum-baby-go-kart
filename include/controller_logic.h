#pragma once
#include <stdint.h>

// Controller input logic: stick centre calibration, arming, mode-click gesture.
// Pure, no Arduino deps — host-testable (test/test_safety).

// Button bits as packed into CtrlPacket.buttons (1 = pressed).
#define BTN_LEFT   0x01   // yellow L, top: speed down
#define BTN_RIGHT  0x02   // yellow R, top: speed up
#define BTN_LJOY   0x04   // left stick click: cycle mode (on release)
#define BTN_RJOY   0x08   // right stick click
#define BTN_ESTOP  (BTN_LJOY | BTN_RJOY)   // both stick clicks held = e-stop

// ---------------- Stick centre calibration ----------------
// The centre used to be whatever the sticks read at power-on. A stick held at
// boot (a kid grabbing the controller) became "centre", so releasing it read as
// full deflection the other way and the cart drove with nobody touching it.
//
// Now the centre is accepted only from a set of `need` consecutive readings where
// every axis averages within `tol` of THIS UNIT's measured rest position
// (`rest[]`, per axis) AND moved less than `maxSpread` across the set (sticks at
// rest). Anything else rejects the set and starts over, so the controller keeps
// trying until the sticks are released.
//
// Why per-unit rest and a small tol (audit 2026-10-05): with tol measured from
// mid-scale (2048) it had to be large (this unit rests up to ~130 counts off
// mid-scale), and a stick held LIGHTLY at power-on still became the centre: on
// release the cart crept (vx 66 / omega 48 at 50% speed). If tol + the rest's own
// drift stays under the deadzone, any accepted centre lies within the deadzone of
// the true rest, so a light hold can never produce motion after release
// (static_assert in config_controller.h).
// Axis order is the caller's (the firmware uses L horiz, L vert, R horiz, R vert).
// Zero-initialized state is ready to use.
struct StickCal {
  uint32_t sum[4];
  uint16_t lo[4], hi[4];
  uint8_t  n;
};

// Returns 0 while collecting, +1 when accepted (center[] written), -1 when the
// set was rejected (collection restarts on the next call).
static inline int stickCalAdd(StickCal* c, const uint16_t raw[4], uint8_t need,
                              const uint16_t rest[4], uint16_t tol, uint16_t maxSpread,
                              uint16_t center[4]) {
  if (c->n == 0) {
    for (int i = 0; i < 4; i++) { c->sum[i] = 0; c->lo[i] = 0xFFFF; c->hi[i] = 0; }
  }
  for (int i = 0; i < 4; i++) {
    c->sum[i] += raw[i];
    if (raw[i] < c->lo[i]) c->lo[i] = raw[i];
    if (raw[i] > c->hi[i]) c->hi[i] = raw[i];
  }
  if (++c->n < need) return 0;

  uint16_t avg[4];
  bool good = true;
  for (int i = 0; i < 4; i++) {
    avg[i] = (uint16_t)(c->sum[i] / c->n);
    int32_t off = (int32_t)avg[i] - (int32_t)rest[i];
    if (off < 0) off = -off;
    if (off > tol || (uint16_t)(c->hi[i] - c->lo[i]) > maxSpread) good = false;
  }
  c->n = 0;
  if (!good) return -1;
  for (int i = 0; i < 4; i++) center[i] = avg[i];
  return 1;
}

// Refine the centre while the controller is disarmed. A set taken off a LIGHTLY
// held stick can pass the tolerance; released, that stick then reads as a
// deflection past the deadzone, so the controller can never re-arm and sits on
// CENTER until rebooted. While disarmed the firmware keeps calibrating and adopts a
// newly accepted centre per axis, but only where it is CLOSER to the unit's
// measured rest than the current one. A stick held while disarmed therefore never
// replaces its true rest position, and the stuck case clears as soon as the
// sticks rest. Returns true if any axis moved.
static inline bool centerImprove(uint16_t cur[4], const uint16_t cand[4], const uint16_t rest[4]) {
  bool moved = false;
  for (int i = 0; i < 4; i++) {
    int32_t dc = (int32_t)cur[i]  - (int32_t)rest[i];
    int32_t dn = (int32_t)cand[i] - (int32_t)rest[i];
    if (dc < 0) dc = -dc;
    if (dn < 0) dn = -dn;
    if (dn < dc) { cur[i] = cand[i]; moved = true; }
  }
  return moved;
}

// ---------------- Arming ----------------
// The controller sends drive commands only while ARMED. It starts disarmed, and
// the e-stop gesture or a joystick bus fault disarms it. It re-arms once the
// e-stop is released AND every stick axis reads neutral (inside the deadzone).
// While disarmed it sends the e-stop flag, so the robot hard-stops and holds.
// This latches the e-stop: letting go of the clicks with a stick still pushed no
// longer launches the cart at that stick.
static inline bool sticksNeutral(int16_t a, int16_t b, int16_t c, int16_t d) {
  return a == 0 && b == 0 && c == 0 && d == 0;
}

static inline bool armStep(bool armed, bool estopHeld, bool neutral) {
  if (estopHeld) return false;
  return armed || neutral;
}

// ---------------- Mode-click gesture ----------------
// Left stick click cycles the feature mode, but the e-stop is BOTH clicks: a human
// rarely lands two clicks inside one 20 ms tick, so cycling on the left PRESS
// changed the mode on about every other e-stop (left landing first), and a few
// stops walked FULL -> RAW. Cycle on the left RELEASE instead, and only when the
// right click never went down during that press.
struct ModeClick { bool pending; };

// Returns true on the tick the mode should advance.
static inline bool modeClickStep(ModeClick* s, uint8_t btnNow, uint8_t btnPrev) {
  bool l  = (btnNow  & BTN_LJOY) != 0;
  bool lp = (btnPrev & BTN_LJOY) != 0;
  bool r  = (btnNow  & BTN_RJOY) != 0;
  if (l && !lp) s->pending = !r;     // fresh press, unless right is already down
  if (l && r)   s->pending = false;  // right joined: this is the e-stop gesture
  if (!l && lp) {
    bool fire = s->pending;
    s->pending = false;
    return fire;
  }
  return false;
}
