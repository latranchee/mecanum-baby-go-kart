// Host (native) unit tests for the safety logic: robot link gate + drive fault
// (safety.h) and controller calibration / arming / mode gesture
// (controller_logic.h). Run with: pio test -e native
#include <unity.h>
#include "safety.h"
#include "controller_logic.h"

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
// Gates as shipped: 30% of 1023 PWM, 2% of ~8500 tps, 300 ms, 10 ms ticks.
#define FPWM 307.0f
#define FTPS 170.0f

static void test_fault_trips_dead_encoder_after_300ms(void) {
  // Wheel at 700 PWM reading zero ticks: trips on the 30th tick (300 ms), not before.
  DriveFault f = {};
  for (int k = 0; k < 29; k++)
    TEST_ASSERT_FALSE(driveFaultStep(&f, 700, 0, FPWM, FTPS, 300, 10));
  TEST_ASSERT_TRUE(driveFaultStep(&f, 700, 0, FPWM, FTPS, 300, 10));
}

static void test_fault_ignores_low_pwm(void) {
  // Below the PWM gate a stopped wheel is just a gentle command (breakaway).
  DriveFault f = {};
  for (int k = 0; k < 100; k++)
    TEST_ASSERT_FALSE(driveFaultStep(&f, 200, 0, FPWM, FTPS, 300, 10));
}

static void test_fault_ignores_moving_wheel(void) {
  DriveFault f = {};
  for (int k = 0; k < 100; k++)
    TEST_ASSERT_FALSE(driveFaultStep(&f, 1000, 2000, FPWM, FTPS, 300, 10));
}

static void test_fault_sign_independent(void) {
  DriveFault a = {}, b = {};
  bool ta = false, tb = false;
  for (int k = 0; k < 30; k++) {
    ta = driveFaultStep(&a, -700, 0, FPWM, FTPS, 300, 10);       // reverse, stalled
    tb = driveFaultStep(&b, -700, -2000, FPWM, FTPS, 300, 10);   // reverse, moving
  }
  TEST_ASSERT_TRUE(ta);
  TEST_ASSERT_FALSE(tb);
}

static void test_fault_timer_resets_on_motion(void) {
  // Any tick of real motion restarts the window: 25 stalled + 1 moving + 25
  // stalled never reaches 300 ms continuous.
  DriveFault f = {};
  bool t = false;
  for (int k = 0; k < 25; k++) t = driveFaultStep(&f, 700, 0, FPWM, FTPS, 300, 10);
  t = driveFaultStep(&f, 700, 1500, FPWM, FTPS, 300, 10);
  for (int k = 0; k < 25; k++) t = driveFaultStep(&f, 700, 0, FPWM, FTPS, 300, 10);
  TEST_ASSERT_FALSE(t);
}

// ---------------- stickCalAdd / centerImprove ----------------
// As shipped: 16 samples, nominal 2048, tolerance 400, spread 60.
static uint16_t calOut[4];

static int calFeed(StickCal* c, uint16_t lh, uint16_t lv, uint16_t rh, uint16_t rv, int n) {
  uint16_t raw[4] = { lh, lv, rh, rv };
  int r = 0;
  for (int k = 0; k < n; k++) r = stickCalAdd(c, raw, 16, 2048, 400, 60, calOut);
  return r;
}

static void test_cal_collecting_returns_zero(void) {
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(0, calFeed(&c, 2040, 2060, 2050, 2030, 15));
}

static void test_cal_accepts_sticks_at_rest(void) {
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(1, calFeed(&c, 2040, 2060, 2050, 2030, 16));
  TEST_ASSERT_EQUAL_UINT16(2040, calOut[0]);
  TEST_ASSERT_EQUAL_UINT16(2060, calOut[1]);
  TEST_ASSERT_EQUAL_UINT16(2050, calOut[2]);
  TEST_ASSERT_EQUAL_UINT16(2030, calOut[3]);
}

static void test_cal_rejects_stick_held_at_poweron(void) {
  // THE REGRESSION: left stick held fully forward while the controller boots.
  // The old code made that the centre, so releasing it read as full deflection
  // the other way and the cart drove by itself.
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(-1, calFeed(&c, 2048, 4000, 2048, 2048, 16));
}

static void test_cal_rejects_moving_stick(void) {
  StickCal c = {};
  int r = 0;
  for (int k = 0; k < 16; k++) {
    uint16_t v = (k & 1) ? 2000 : 2100;   // 100-count swing > 60 spread
    uint16_t raw[4] = { 2048, v, 2048, 2048 };
    r = stickCalAdd(&c, raw, 16, 2048, 400, 60, calOut);
  }
  TEST_ASSERT_EQUAL_INT(-1, r);
}

static void test_cal_retries_until_released(void) {
  StickCal c = {};
  TEST_ASSERT_EQUAL_INT(-1, calFeed(&c, 2048, 4000, 2048, 2048, 16));  // held
  TEST_ASSERT_EQUAL_INT(1,  calFeed(&c, 2045, 2052, 2048, 2049, 16));  // released
  TEST_ASSERT_EQUAL_UINT16(2052, calOut[1]);
}

static void test_center_improve_moves_only_toward_nominal(void) {
  uint16_t cur[4]  = { 2300, 2048, 2048, 2100 };
  uint16_t cand[4] = { 2060, 2400, 2048, 2090 };
  TEST_ASSERT_TRUE(centerImprove(cur, cand, 2048));
  TEST_ASSERT_EQUAL_UINT16(2060, cur[0]);   // closer: adopted
  TEST_ASSERT_EQUAL_UINT16(2048, cur[1]);   // a held stick (farther): never adopted
  TEST_ASSERT_EQUAL_UINT16(2048, cur[2]);
  TEST_ASSERT_EQUAL_UINT16(2090, cur[3]);
  uint16_t same[4] = { 2060, 2048, 2048, 2090 };
  TEST_ASSERT_FALSE(centerImprove(cur, same, 2048));
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
  RUN_TEST(test_fault_sign_independent);
  RUN_TEST(test_fault_timer_resets_on_motion);
  RUN_TEST(test_cal_collecting_returns_zero);
  RUN_TEST(test_cal_accepts_sticks_at_rest);
  RUN_TEST(test_cal_rejects_stick_held_at_poweron);
  RUN_TEST(test_cal_rejects_moving_stick);
  RUN_TEST(test_cal_retries_until_released);
  RUN_TEST(test_center_improve_moves_only_toward_nominal);
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
