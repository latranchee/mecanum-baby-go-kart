// Host (native) unit tests for the safety logic: robot link gate, drive fault and
// drive gate (safety.h), and controller calibration / arming / mode gesture
// (controller_logic.h). Run with: pio test -e native
#include <unity.h>
#include "safety.h"
#include "controller_logic.h"
#include "control_math.h"       // normalize()
#include "config_controller.h"  // CENTER_REST / TOL / DRIFT, DEADZONE_RAW, HALF_RANGE
#include "config_robot.h"       // FAULT_* as shipped

// ---------------- linkAgeMs ----------------

static void test_link_age_normal(void) {
  TEST_ASSERT_EQUAL_UINT32(30, linkAgeMs(1030, 1000));
}

static void test_link_age_future_stamp_is_fresh(void) {
  // THE REGRESSION: onRecv (core 0) stamped the packet 1 ms after the drive loop
  // (core 1) read `now`. Unsigned now - last wrapped to ~4e9 = "dead link" = a
  // one-tick hard stop. A stamp from the future is simply fresh.
  TEST_ASSERT_EQUAL_UINT32(0, linkAgeMs(1000, 1001));
}

static void test_link_age_across_millis_wrap(void) {
  TEST_ASSERT_EQUAL_UINT32(20, linkAgeMs(10, 0xFFFFFFF6u));
}

// ---------------- linkGateCheck ----------------
static const uint8_t MAC_A[6] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60 };
static const uint8_t MAC_B[6] = { 0x99, 0x88, 0x77, 0x66, 0x55, 0x44 };
#define RESYNC 500u

static void test_gate_accepts_newer_drops_dup_and_reorder(void) {
  LinkGate g = {};
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 1, 1000, RESYNC));
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 2, 1020, RESYNC));
  TEST_ASSERT_EQUAL(LINK_STALE,  linkGateCheck(&g, MAC_A, 2, 1040, RESYNC));  // duplicate
  TEST_ASSERT_EQUAL(LINK_STALE,  linkGateCheck(&g, MAC_A, 1, 1040, RESYNC));  // reordered
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 3, 1060, RESYNC));
}

static void test_gate_controller_reboot_soon_after_poweron(void) {
  // THE REGRESSION: the controller ran 15 s (seq 750) and was power-cycled; it
  // restarts at seq 1. The old rule resynced only on a backward jump >= 1000, so
  // the robot dropped the new session for ~15 s while the controller showed
  // ONLINE. Its reboot is longer than the watchdog, so the silence reopens the gate.
  LinkGate g = {};
  for (uint32_t s = 1; s <= 750; s++) linkGateCheck(&g, MAC_A, s, s * 20, RESYNC);
  uint32_t t = 750 * 20 + 1500;
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 1, t, RESYNC));
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 2, t + 20, RESYNC));
}

static void test_gate_backward_jump_on_live_link_is_stale(void) {
  // While the link is live a big backward jump is an old frame, never a resync.
  LinkGate g = {};
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 5000, 1000, RESYNC));
  TEST_ASSERT_EQUAL(LINK_STALE,  linkGateCheck(&g, MAC_A, 1,    1020, RESYNC));
}

static void test_gate_second_sender_dropped_while_live(void) {
  LinkGate g = {};
  TEST_ASSERT_EQUAL(LINK_ACCEPT,  linkGateCheck(&g, MAC_A, 10,  1000, RESYNC));
  TEST_ASSERT_EQUAL(LINK_FOREIGN, linkGateCheck(&g, MAC_B, 999, 1010, RESYNC));
  TEST_ASSERT_EQUAL(LINK_ACCEPT,  linkGateCheck(&g, MAC_A, 11,  1020, RESYNC));
}

static void test_gate_other_sender_takes_over_after_silence(void) {
  LinkGate g = {};
  TEST_ASSERT_EQUAL(LINK_ACCEPT,  linkGateCheck(&g, MAC_A, 10, 1000, RESYNC));
  TEST_ASSERT_EQUAL(LINK_ACCEPT,  linkGateCheck(&g, MAC_B, 3,  1600, RESYNC));  // A silent 600 ms
  TEST_ASSERT_EQUAL(LINK_FOREIGN, linkGateCheck(&g, MAC_A, 11, 1620, RESYNC));  // B owns it now
}

static void test_gate_seq_wrap_is_newer(void) {
  LinkGate g = {};
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 0xFFFFFFFFu, 1000, RESYNC));
  TEST_ASSERT_EQUAL(LINK_ACCEPT, linkGateCheck(&g, MAC_A, 0,           1020, RESYNC));
}

// ---------------- driveFaultStep ----------------
// Gates as shipped (config_robot.h): 30% of 1023 PWM, absolute FAULT_TPS, 300 ms.
#define FPWM (FAULT_PWM_FRAC * 1023.0f)
#define FTPS FAULT_TPS
#define FMS  FAULT_MS

static void test_fault_trips_dead_encoder_after_300ms(void) {
  // Wheel at 700 PWM reading zero ticks: trips on the 30th tick (300 ms), not before.
  DriveFault f = {};
  for (int k = 0; k < 29; k++)
    TEST_ASSERT_FALSE(driveFaultStep(&f, 700, 0, FPWM, FTPS, FMS, 10));
  TEST_ASSERT_TRUE(driveFaultStep(&f, 700, 0, FPWM, FTPS, FMS, 10));
}

static void test_fault_ignores_low_pwm(void) {
  // Below the PWM gate a stopped wheel is just a gentle command (breakaway).
  DriveFault f = {};
  for (int k = 0; k < 100; k++)
    TEST_ASSERT_FALSE(driveFaultStep(&f, 200, 0, FPWM, FTPS, FMS, 10));
}

static void test_fault_ignores_moving_wheel(void) {
  DriveFault f = {};
  for (int k = 0; k < 100; k++)
    TEST_ASSERT_FALSE(driveFaultStep(&f, 1000, 2000, FPWM, FTPS, FMS, 10));
}

static void test_fault_slow_heavy_start_does_not_trip(void) {
  // THE REGRESSION (review 2026-10-05): the threshold was 2% of each wheel's own
  // MAX_TPS, 248 tps on the fast pair after recalibration, so a heavy start already
  // rolling at ~221 tps under high PWM tripped and stopped the cart. Absolute now.
  DriveFault f = {};
  for (int k = 0; k < 100; k++)
    TEST_ASSERT_FALSE(driveFaultStep(&f, 800, 221, FPWM, FTPS, FMS, 10));
}

static void test_fault_jittering_jam_still_trips(void) {
  // A jammed wheel vibrating +-1 count per 10 ms tick (+-100 tps) is still stopped.
  DriveFault f = {};
  bool t = false;
  for (int k = 0; k < 30; k++) t = driveFaultStep(&f, 700, (k & 1) ? 100.0f : -100.0f, FPWM, FTPS, FMS, 10);
  TEST_ASSERT_TRUE(t);
}

static void test_fault_sign_independent(void) {
  DriveFault a = {}, b = {};
  bool ta = false, tb = false;
  for (int k = 0; k < 30; k++) {
    ta = driveFaultStep(&a, -700, 0, FPWM, FTPS, FMS, 10);       // reverse, stalled
    tb = driveFaultStep(&b, -700, -2000, FPWM, FTPS, FMS, 10);   // reverse, moving
  }
  TEST_ASSERT_TRUE(ta);
  TEST_ASSERT_FALSE(tb);
}

static void test_fault_timer_resets_on_motion(void) {
  // Any tick of real motion restarts the window: 25 stalled + 1 moving + 25
  // stalled never reaches 300 ms continuous.
  DriveFault f = {};
  bool t = false;
  for (int k = 0; k < 25; k++) t = driveFaultStep(&f, 700, 0, FPWM, FTPS, FMS, 10);
  t = driveFaultStep(&f, 700, 1500, FPWM, FTPS, FMS, 10);
  for (int k = 0; k < 25; k++) t = driveFaultStep(&f, 700, 0, FPWM, FTPS, FMS, 10);
  TEST_ASSERT_FALSE(t);
}

// ---------------- driveGateStep ----------------
static DriveGateIn gIn(bool linkOk, bool estop, bool neutral, bool testMode = false) {
  DriveGateIn in;
  in.encReady = true; in.linkOk = linkOk; in.estop = estop;
  in.neutral = neutral; in.testMode = testMode;
  return in;
}

static void test_gate_boot_waits_for_neutral_frame(void) {
  DriveGate g = { false, true, 0 };   // as shipped: latched until the first neutral frame
  TEST_ASSERT_TRUE(driveGateStep(&g, gIn(false, false, false), 0, 1000).stop);  // no link
  TEST_ASSERT_TRUE(driveGateStep(&g, gIn(true, false, false), 10, 1000).stop);  // stick held
  DriveGateOut o = driveGateStep(&g, gIn(true, false, true), 20, 1000);         // neutral
  TEST_ASSERT_TRUE(o.linkReleased);
  TEST_ASSERT_FALSE(o.stop);
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, false), 30, 1000).stop); // now drives
}

static void test_gate_link_loss_needs_neutral_to_resume(void) {
  // THE REGRESSION (review 2026-10-05): a marginal link stopped the cart and then
  // relaunched it at whatever the stick held, over and over.
  DriveGate g = { false, false, 0 };
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, false), 0, 1000).stop);  // driving
  DriveGateOut o = driveGateStep(&g, gIn(false, false, false), 10, 1000);       // link lost
  TEST_ASSERT_TRUE(o.stop);
  TEST_ASSERT_TRUE(o.linkLatched);
  TEST_ASSERT_TRUE(driveGateStep(&g, gIn(true, false, false), 20, 1000).stop);  // back, stick held
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, true), 30, 1000).stop);  // neutral
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, false), 40, 1000).stop); // drives again
}

static void test_gate_estop_releases_link_latch(void) {
  DriveGate g = { false, true, 0 };
  DriveGateOut o = driveGateStep(&g, gIn(true, true, false), 0, 1000);   // e-stop frame
  TEST_ASSERT_TRUE(o.stop);           // e-stop itself still stops
  TEST_ASSERT_TRUE(o.linkReleased);
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, false), 10, 1000).stop);
}

static void test_gate_test_mode_ignores_link_latch(void) {
  // Serial test mode has its own link watchdog; the radio latch must not block `t`.
  DriveGate g = { false, true, 0 };
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, false, true), 0, 1000).stop);
  TEST_ASSERT_FALSE(g.linkLatch);
}

static void test_gate_wheel_fault_does_not_stop_the_drive(void) {
  // Olivier 2026-10-05: a faulted wheel is switched off and reported; the other
  // wheels keep driving. The gate must not stop the drive for it.
  DriveGate g = { false, false, 0 };
  driveGateTrip(&g, 0);
  for (uint32_t t = 0; t <= 3000; t += 100)
    TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, false), t, 1000).stop);  // stick held
  TEST_ASSERT_TRUE(g.fault);   // still latched: the wheel stays off while driving
}

static void test_gate_fault_rearms_only_after_neutral_window(void) {
  DriveGate g = { false, false, 0 };
  driveGateTrip(&g, 0);
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, false), 500, 1000).faultCleared); // held: restarts window
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, true), 600, 1000).faultCleared);  // neutral from 500
  TEST_ASSERT_FALSE(driveGateStep(&g, gIn(true, false, true), 1499, 1000).faultCleared); // 999 ms
  DriveGateOut o = driveGateStep(&g, gIn(true, false, true), 1500, 1000);                // 1000 ms
  TEST_ASSERT_TRUE(o.faultCleared);
  TEST_ASSERT_FALSE(g.fault);
}

static void test_gate_fault_not_cleared_without_link(void) {
  // Neutral-looking but stale frames (link lost) never re-arm a wheel.
  DriveGate g = { false, false, 0 };
  driveGateTrip(&g, 0);
  for (uint32_t t = 0; t <= 3000; t += 100)
    TEST_ASSERT_TRUE(driveGateStep(&g, gIn(false, false, true), t, 1000).stop);   // link down
  TEST_ASSERT_TRUE(g.fault);
}

// ---------------- WheelRecovery ----------------
static const WheelRecoveryCfg RCFG = { FAULT_RETRY_FIRST_MS, FAULT_RETRY_MAX_MS,
                                       FAULT_RETRY_RESET_MS, FAULT_TPS, FAULT_ROLL_MS };

// Step an off wheel (not rolling) until it re-arms; returns the ms it took.
static uint32_t untilRearm(WheelRecovery* r, uint32_t start, int* why) {
  for (uint32_t t = start; t < start + 60000; t += 10) {
    int w = wheelRecoveryStep(r, t, 0.0f, 10.0f, RCFG);
    if (w) { *why = w; return t - start; }
  }
  *why = 0;
  return 0;
}

static void test_recovery_comes_back_while_driving(void) {
  // THE REGRESSION (Olivier 2026-10-05): "once it's declared dead it doesn't come
  // back ever". A switched-off wheel is retried after FAULT_RETRY_FIRST_MS with no
  // operator action at all.
  WheelRecovery r = {};
  wheelRecoveryTrip(&r, 1000, RCFG);
  int why = 0;
  TEST_ASSERT_UINT32_WITHIN(10, FAULT_RETRY_FIRST_MS, untilRearm(&r, 1000, &why));
  TEST_ASSERT_EQUAL_INT(2, why);
}

static void test_recovery_backoff_doubles_then_caps(void) {
  // A dead encoder re-trips right after every retry: delays 1, 2, 4, 8, 8 s.
  WheelRecovery r = {};
  uint32_t t = 0;
  const uint32_t expect[] = { 1000, 2000, 4000, 8000, 8000 };
  for (uint32_t e : expect) {
    wheelRecoveryTrip(&r, t, RCFG);
    int why = 0;
    uint32_t took = untilRearm(&r, t, &why);
    TEST_ASSERT_UINT32_WITHIN(10, e, took);
    t += took;
    wheelRecoveryRearm(&r, t);
    t += 400;                                 // re-trips ~0.4 s after the retry
  }
}

static void test_recovery_backoff_resets_after_healthy_run(void) {
  WheelRecovery r = {};
  wheelRecoveryTrip(&r, 0, RCFG);
  int why = 0;
  uint32_t t = untilRearm(&r, 0, &why);
  wheelRecoveryRearm(&r, t);
  wheelRecoveryTrip(&r, t + 400, RCFG);       // quick re-trip: 2 s
  TEST_ASSERT_EQUAL_UINT32(2000, r.delayMs);
  t += 400 + untilRearm(&r, t + 400, &why);
  wheelRecoveryRearm(&r, t);
  wheelRecoveryTrip(&r, t + FAULT_RETRY_RESET_MS + 100, RCFG);   // healthy long enough
  TEST_ASSERT_EQUAL_UINT32(FAULT_RETRY_FIRST_MS, r.delayMs);
}

static void test_recovery_free_roll_rearms_at_once(void) {
  // On the floor a freed wheel rolls with the cart while unpowered: that proves it
  // is no longer jammed AND its encoder works. Back on after FAULT_ROLL_MS.
  WheelRecovery r = {};
  wheelRecoveryTrip(&r, 0, RCFG);
  int why = 0;
  uint32_t t = 0;
  for (; t < 1000; t += 10) {
    why = wheelRecoveryStep(&r, t, 600.0f, 10.0f, RCFG);
    if (why) break;
  }
  TEST_ASSERT_EQUAL_INT(1, why);
  TEST_ASSERT_TRUE(t <= (uint32_t)FAULT_ROLL_MS + 10);
}

static void test_recovery_ignores_jitter(void) {
  // +-1 count jitter (100 tps) on a jammed wheel is not "rolling freely".
  WheelRecovery r = {};
  wheelRecoveryTrip(&r, 0, RCFG);
  for (uint32_t t = 0; t < 900; t += 10)
    TEST_ASSERT_EQUAL_INT(0, wheelRecoveryStep(&r, t, (t / 10) & 1 ? 100.0f : -100.0f, 10.0f, RCFG));
}

// ---------------- faultLabel ----------------

static void test_fault_label_rider_names(void) {
  char b[24];
  faultLabel(0x00, b, sizeof(b)); TEST_ASSERT_EQUAL_STRING("", b);
  faultLabel(0x01, b, sizeof(b)); TEST_ASSERT_EQUAL_STRING("FAULT RR", b);   // slot 0 = rider RR
  faultLabel(0x08, b, sizeof(b)); TEST_ASSERT_EQUAL_STRING("FAULT FL", b);   // slot 3 = rider FL
  faultLabel(0x05, b, sizeof(b)); TEST_ASSERT_EQUAL_STRING("FAULT RR FR", b);
  faultLabel(0x0F, b, sizeof(b)); TEST_ASSERT_EQUAL_STRING("FAULT RR RL FR FL", b);
  faultLabel(0xF0, b, sizeof(b)); TEST_ASSERT_EQUAL_STRING("", b);           // only 4 slots exist
}

static void test_fault_label_truncates_safely(void) {
  char b[9];
  faultLabel(0x0F, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING("FAULT RR", b);   // 8 chars + NUL, never overruns
}

static void test_gate_encoders_down_always_stops(void) {
  DriveGate g = { false, false, 0 };
  DriveGateIn in = gIn(true, false, false);
  in.encReady = false;
  TEST_ASSERT_TRUE(driveGateStep(&g, in, 0, 1000).stop);
}

// ---------------- stickCalAdd / centerImprove ----------------
// As shipped: CAL_SAMPLES samples within CENTER_TOL of this unit's CENTER_REST.
static uint16_t calOut[4];
#define RLH CENTER_REST[0]
#define RLV CENTER_REST[1]
#define RRH CENTER_REST[2]
#define RRV CENTER_REST[3]

static int calFeed(StickCal* c, uint16_t lh, uint16_t lv, uint16_t rh, uint16_t rv, int n) {
  uint16_t raw[4] = { lh, lv, rh, rv };
  int r = 0;
  for (int k = 0; k < n; k++)
    r = stickCalAdd(c, raw, CAL_SAMPLES, CENTER_REST, CENTER_TOL, CENTER_SPREAD, calOut);
  return r;
}

static void test_cal_collecting_returns_zero(void) {
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(0, calFeed(&c, RLH, RLV, RRH, RRV, CAL_SAMPLES - 1));
}

static void test_cal_accepts_sticks_at_rest(void) {
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(1, calFeed(&c, RLH + 10, RLV - 5, RRH + 3, RRV - 8, CAL_SAMPLES));
  TEST_ASSERT_EQUAL_UINT16(RLH + 10, calOut[0]);
  TEST_ASSERT_EQUAL_UINT16(RLV - 5,  calOut[1]);
  TEST_ASSERT_EQUAL_UINT16(RRH + 3,  calOut[2]);
  TEST_ASSERT_EQUAL_UINT16(RRV - 8,  calOut[3]);
}

static void test_cal_rejects_stick_held_at_poweron(void) {
  // Left stick held fully forward while the controller boots: the original code
  // made that the centre, so releasing it read as full deflection the other way.
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(-1, calFeed(&c, RLH, 4000, RRH, RRV, CAL_SAMPLES));
}

static void test_cal_rejects_light_hold(void) {
  // THE REGRESSION (review 2026-10-05): with a 400-count tolerance around 2048 a
  // stick held lightly (~20%) was accepted and the cart crept after release.
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(-1, calFeed(&c, RLH, RLV + 100, RRH, RRV, CAL_SAMPLES));
}

static void test_cal_rejects_moving_stick(void) {
  StickCal c = {};
  int r = 0;
  for (int k = 0; k < CAL_SAMPLES; k++) {
    uint16_t v = (k & 1) ? RLV - 40 : RLV + 40;   // 80-count swing > CENTER_SPREAD
    uint16_t raw[4] = { RLH, v, RRH, RRV };
    r = stickCalAdd(&c, raw, CAL_SAMPLES, CENTER_REST, CENTER_TOL, CENTER_SPREAD, calOut);
  }
  TEST_ASSERT_EQUAL_INT(-1, r);
}

static void test_cal_retries_until_released(void) {
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(-1, calFeed(&c, RLH, 4000, RRH, RRV, CAL_SAMPLES));        // held
  TEST_ASSERT_EQUAL_INT(1,  calFeed(&c, RLH - 3, RLV + 4, RRH, RRV + 1, CAL_SAMPLES));  // released
  TEST_ASSERT_EQUAL_UINT16(RLV + 4, calOut[1]);
}

static void test_cal_light_hold_can_never_move_the_cart(void) {
  // THE GUARANTEE behind CENTER_TOL: for EVERY centre the calibration can accept
  // (within CENTER_TOL of rest) and every true rest within CENTER_DRIFT, the
  // released stick normalizes to exactly 0. Exhaustive over both offsets.
  for (int off = -(int)CENTER_TOL; off <= (int)CENTER_TOL; off++) {
    for (int drift = -(int)CENTER_DRIFT; drift <= (int)CENTER_DRIFT; drift++) {
      uint16_t accepted = (uint16_t)(RLV + off);
      uint16_t released = (uint16_t)(RLV + drift);
      TEST_ASSERT_EQUAL_INT16(0, normalize(released, accepted, DEADZONE_RAW, HALF_RANGE));
    }
  }
}

static void test_center_improve_moves_only_toward_rest(void) {
  uint16_t cur[4]  = { (uint16_t)(RLH + 40), RLV, RRH, (uint16_t)(RRV + 30) };
  uint16_t cand[4] = { (uint16_t)(RLH + 5), (uint16_t)(RLV + 40), RRH, (uint16_t)(RRV - 20) };
  TEST_ASSERT_TRUE(centerImprove(cur, cand, CENTER_REST));
  TEST_ASSERT_EQUAL_UINT16(RLH + 5,  cur[0]);   // closer: adopted
  TEST_ASSERT_EQUAL_UINT16(RLV,      cur[1]);   // a held stick (farther): never adopted
  TEST_ASSERT_EQUAL_UINT16(RRH,      cur[2]);
  TEST_ASSERT_EQUAL_UINT16(RRV - 20, cur[3]);
  uint16_t same[4] = { cur[0], cur[1], cur[2], cur[3] };
  TEST_ASSERT_FALSE(centerImprove(cur, same, CENTER_REST));
}

// ---------------- armStep ----------------

static void test_arm_boot_needs_neutral(void) {
  TEST_ASSERT_FALSE(armStep(false, false, false));   // stick pushed at boot
  TEST_ASSERT_TRUE(armStep(false, false, true));     // centred -> armed
}

static void test_arm_estop_latches_until_neutral(void) {
  // THE LATCH: e-stop disarms; letting go of the clicks with a stick still pushed
  // stays disarmed; only neutral sticks re-arm.
  bool a = true;
  a = armStep(a, true,  false);  TEST_ASSERT_FALSE(a);   // e-stop held
  a = armStep(a, false, false);  TEST_ASSERT_FALSE(a);   // released, stick pushed
  a = armStep(a, true,  true);   TEST_ASSERT_FALSE(a);   // clicks held, sticks centred
  a = armStep(a, false, true);   TEST_ASSERT_TRUE(a);    // released + centred
  a = armStep(a, false, false);  TEST_ASSERT_TRUE(a);    // armed: driving is fine
}

static void test_sticks_neutral(void) {
  TEST_ASSERT_TRUE(sticksNeutral(0, 0, 0, 0));
  TEST_ASSERT_FALSE(sticksNeutral(0, 0, 1, 0));
}

// ---------------- modeClickStep ----------------

static void test_mode_click_fires_on_release_only(void) {
  ModeClick s = {};
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_LJOY, 0));         // press
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_LJOY, BTN_LJOY));  // held
  TEST_ASSERT_TRUE(modeClickStep(&s, 0, BTN_LJOY));          // release -> cycle
}

static void test_mode_click_left_first_estop_never_cycles(void) {
  // THE REGRESSION: e-stop with the left click landing one tick before the right
  // used to cycle the mode on the left press.
  ModeClick s = {};
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_LJOY, 0));
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_ESTOP, BTN_LJOY));
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_ESTOP, BTN_ESTOP));
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_RJOY, BTN_ESTOP));  // left released first
  TEST_ASSERT_FALSE(modeClickStep(&s, 0, BTN_RJOY));
}

static void test_mode_click_right_first_estop_never_cycles(void) {
  ModeClick s = {};
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_RJOY, 0));
  TEST_ASSERT_FALSE(modeClickStep(&s, BTN_ESTOP, BTN_RJOY));
  TEST_ASSERT_FALSE(modeClickStep(&s, 0, BTN_ESTOP));          // both released together
}

static void test_mode_click_right_tap_during_hold_cancels(void) {
  ModeClick s = {};
  modeClickStep(&s, BTN_LJOY, 0);
  modeClickStep(&s, BTN_ESTOP, BTN_LJOY);    // right joins
  modeClickStep(&s, BTN_LJOY, BTN_ESTOP);    // right lets go, left still held
  TEST_ASSERT_FALSE(modeClickStep(&s, 0, BTN_LJOY));
}

static void test_mode_click_ignores_other_buttons(void) {
  ModeClick s = {};
  modeClickStep(&s, BTN_LJOY | BTN_RIGHT, 0);   // speed-up tap during the click
  TEST_ASSERT_TRUE(modeClickStep(&s, 0, BTN_LJOY | BTN_RIGHT));
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_link_age_normal);
  RUN_TEST(test_link_age_future_stamp_is_fresh);
  RUN_TEST(test_link_age_across_millis_wrap);
  RUN_TEST(test_gate_accepts_newer_drops_dup_and_reorder);
  RUN_TEST(test_gate_controller_reboot_soon_after_poweron);
  RUN_TEST(test_gate_backward_jump_on_live_link_is_stale);
  RUN_TEST(test_gate_second_sender_dropped_while_live);
  RUN_TEST(test_gate_other_sender_takes_over_after_silence);
  RUN_TEST(test_gate_seq_wrap_is_newer);
  RUN_TEST(test_fault_trips_dead_encoder_after_300ms);
  RUN_TEST(test_fault_ignores_low_pwm);
  RUN_TEST(test_fault_ignores_moving_wheel);
  RUN_TEST(test_fault_slow_heavy_start_does_not_trip);
  RUN_TEST(test_fault_jittering_jam_still_trips);
  RUN_TEST(test_fault_sign_independent);
  RUN_TEST(test_fault_timer_resets_on_motion);
  RUN_TEST(test_gate_boot_waits_for_neutral_frame);
  RUN_TEST(test_gate_link_loss_needs_neutral_to_resume);
  RUN_TEST(test_gate_estop_releases_link_latch);
  RUN_TEST(test_gate_test_mode_ignores_link_latch);
  RUN_TEST(test_gate_wheel_fault_does_not_stop_the_drive);
  RUN_TEST(test_gate_fault_rearms_only_after_neutral_window);
  RUN_TEST(test_gate_fault_not_cleared_without_link);
  RUN_TEST(test_recovery_comes_back_while_driving);
  RUN_TEST(test_recovery_backoff_doubles_then_caps);
  RUN_TEST(test_recovery_backoff_resets_after_healthy_run);
  RUN_TEST(test_recovery_free_roll_rearms_at_once);
  RUN_TEST(test_recovery_ignores_jitter);
  RUN_TEST(test_fault_label_rider_names);
  RUN_TEST(test_fault_label_truncates_safely);
  RUN_TEST(test_gate_encoders_down_always_stops);
  RUN_TEST(test_cal_collecting_returns_zero);
  RUN_TEST(test_cal_accepts_sticks_at_rest);
  RUN_TEST(test_cal_rejects_stick_held_at_poweron);
  RUN_TEST(test_cal_rejects_light_hold);
  RUN_TEST(test_cal_rejects_moving_stick);
  RUN_TEST(test_cal_retries_until_released);
  RUN_TEST(test_cal_light_hold_can_never_move_the_cart);
  RUN_TEST(test_center_improve_moves_only_toward_rest);
  RUN_TEST(test_arm_boot_needs_neutral);
  RUN_TEST(test_arm_estop_latches_until_neutral);
  RUN_TEST(test_sticks_neutral);
  RUN_TEST(test_mode_click_fires_on_release_only);
  RUN_TEST(test_mode_click_left_first_estop_never_cycles);
  RUN_TEST(test_mode_click_right_first_estop_never_cycles);
  RUN_TEST(test_mode_click_right_tap_during_hold_cancels);
  RUN_TEST(test_mode_click_ignores_other_buttons);
  return UNITY_END();
}
