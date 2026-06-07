// Host (native) unit tests for the pure logic headers. Run with: pio test -e native
// These lock the sign/remap/scaling math that has repeatedly needed fixing.
#include <unity.h>
#include <string.h>
#include "kinematics.h"          // mecanumMix + forwardKinematics
#include "control_math.h"
#include "governor.h"           // speedGovernorScale (cross-wheel sync)
#include "body_loop.h"          // bodyCorrection (body-space outer loop)
#include "protocol.h"            // CtrlPacket wire format
#include "config_controller.h"  // CURVE, DEADZONE_RAW, HALF_RANGE
#include "config_robot.h"       // BODY_W_THRESH (BUG-008 blend width)

// ---------------- mecanumMix ----------------
// Slot map: [0]=FL [1]=FR [2]=RL [3]=RR.

static void test_mix_forward_all_equal(void) {
  int32_t c[4];
  mecanumMix(1000, 0, 0, c);
  TEST_ASSERT_EQUAL_INT32(1000, c[0]);
  TEST_ASSERT_EQUAL_INT32(1000, c[1]);
  TEST_ASSERT_EQUAL_INT32(1000, c[2]);
  TEST_ASSERT_EQUAL_INT32(1000, c[3]);
}

static void test_mix_strafe_diagonal_opposite(void) {
  int32_t c[4];
  mecanumMix(0, 1000, 0, c);  // strafe right
  // FL/RR negative, FR/RL positive.
  TEST_ASSERT_EQUAL_INT32(-1000, c[0]);  // FL
  TEST_ASSERT_EQUAL_INT32( 1000, c[1]);  // FR
  TEST_ASSERT_EQUAL_INT32( 1000, c[2]);  // RL
  TEST_ASSERT_EQUAL_INT32(-1000, c[3]);  // RR
}

static void test_mix_spin_left_right_opposite(void) {
  int32_t c[4];
  mecanumMix(0, 0, 1000, c);  // omega CCW+
  // Left wheels (FL,RL) one sign, right wheels (FR,RR) the other.
  TEST_ASSERT_EQUAL_INT32(-1000, c[0]);  // FL
  TEST_ASSERT_EQUAL_INT32( 1000, c[1]);  // FR
  TEST_ASSERT_EQUAL_INT32(-1000, c[2]);  // RL
  TEST_ASSERT_EQUAL_INT32( 1000, c[3]);  // RR
}

static void test_mix_saturation_caps_at_1000(void) {
  int32_t c[4];
  mecanumMix(1000, 1000, 1000, c);  // FR would be 3000 before scaling
  for (int i = 0; i < 4; i++) {
    TEST_ASSERT_TRUE(c[i] <= 1000 && c[i] >= -1000);
  }
  TEST_ASSERT_EQUAL_INT32(1000, c[1]);   // peak wheel pinned to +1000
  TEST_ASSERT_EQUAL_INT32(-333, c[0]);   // -1000 * 1000 / 3000
}

static void test_normalize_quad_scales_over_limit(void) {
  // BUG-007: base+correction can exceed +-1000; normalizeQuad must scale the set
  // back so peak == limit while preserving the ratio (direction unchanged).
  int32_t c[4] = { 1200, -800, 1600, 400 };  // peak 1600
  normalizeQuad(c, 1000);
  TEST_ASSERT_EQUAL_INT32(1000, c[2]);            // 1600 * 1000 / 1600
  TEST_ASSERT_EQUAL_INT32(750,  c[0]);            // 1200 * 1000 / 1600
  TEST_ASSERT_EQUAL_INT32(-500, c[1]);            // -800 * 1000 / 1600
  TEST_ASSERT_EQUAL_INT32(250,  c[3]);            //  400 * 1000 / 1600
}

static void test_normalize_quad_under_limit_untouched(void) {
  int32_t c[4] = { 500, -300, 900, 0 };  // peak 900 <= 1000
  normalizeQuad(c, 1000);
  TEST_ASSERT_EQUAL_INT32(500,  c[0]);
  TEST_ASSERT_EQUAL_INT32(-300, c[1]);
  TEST_ASSERT_EQUAL_INT32(900,  c[2]);
  TEST_ASSERT_EQUAL_INT32(0,    c[3]);
}

// ---------------- speedGovernorScale (cross-wheel sync) ----------------
// Signature: (cmd, meas, outPwm, refTps, pwmMax, loScale, satFrac, valid).
// refTps = uniform target reference (weakest wheel's max). 2100 here. pwmMax=1023.
// loScale=0.1, satFrac=0.90 (=> saturation threshold 920.7) unless noted. ONLY a
// saturated wheel can drag the group, so satFrac is also the engage gate.
#define GMAX 2100.0f
#define GPWM 1023.0f
#define GLO  0.10f
#define GSAT 0.90f

static void test_gov_all_tracking_no_scale(void) {
  // Every wheel at full command, full speed, out pinned (1023) but ratio 1.0 -> 1.0.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   meas[4] = { 2100, 2100, 2100, 2100 };
  float   out[4]  = { 1023, 1023, 1023, 1023 };
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

static void test_gov_held_wheel_pulls_group_to_floor(void) {
  // RL (slot 2) held -> measures 0, out pinned at max -> group collapses to floor.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   meas[4] = { 2100, 2100,    0, 2100 };
  float   out[4]  = { 1023, 1023, 1023, 1023 };
  TEST_ASSERT_EQUAL_FLOAT(0.1f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

static void test_gov_forward_reverse_symmetric_held_wheel(void) {
  // THE REGRESSION: a held wheel must collapse the group in BOTH forward and
  // reverse. The old signed-ratio judge engaged only in reverse (forward phantom
  // read as "keeping up" -> stayed 1.0 -> the free wheel ran away). With a
  // magnitude judge the held wheel's small phantom speed governs the group hard
  // either way. The two need not be bit-identical (wrong-way detection differs by
  // sign), only that BOTH strongly engage near the floor.
  int32_t fwd[4]   = {  1000,  1000,  1000,  1000 };
  int32_t rev[4]   = { -1000, -1000, -1000, -1000 };
  // FL (slot 0) held: phantom reads the SAME small positive speed regardless of
  // command (sign set by encB rest level), out pinned at max.
  float   measF[4] = {  300,  2100,  2100,  2100 };
  float   measR[4] = {  300, -2100, -2100, -2100 };
  float   out[4]   = { 1023, 1023, 1023, 1023 };
  float gF = speedGovernorScale(fwd, measF, out, GMAX, GPWM, GLO, GSAT, 0);
  float gR = speedGovernorScale(rev, measR, out, GMAX, GPWM, GLO, GSAT, 0);
  TEST_ASSERT_TRUE(gF <= 0.2f);   // forward now engages (was 1.0 = the bug)
  TEST_ASSERT_TRUE(gR <= 0.2f);   // reverse still engages
}

static void test_gov_saturated_phantom_still_engages(void) {
  // Saturated wheel (out pinned at max) reaching only ratio 0.7: it physically
  // can't deliver more, so the group slows to 0.7 — saturation is what makes a
  // wheel a limiter.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   meas[4] = { 1470, 2100, 2100, 2100 };   // 1470/2100 = 0.7
  float   out[4]  = { 1023, 1023, 1023, 1023 };   // FL pinned at max
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

static void test_gov_mild_lag_unsaturated_ignored(void) {
  // Ratio 0.8 but NOT saturated (out 818 < 920): has headroom (or is throttled),
  // so it never limits the group regardless of ratio. The anti-latch rule.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   meas[4] = { 2100, 2100, 1680, 2100 };   // 0.8
  float   out[4]  = { 1023, 1023,  818, 1023 };
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

static void test_gov_zero_command_no_demand(void) {
  int32_t cmd[4]  = { 0, 0, 0, 0 };
  float   meas[4] = { 0, 0, 0, 0 };
  float   out[4]  = { 0, 0, 0, 0 };
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

static void test_gov_invalid_wheel_skipped(void) {
  // Open-loop wheel (slot 1) measures 0 but is excluded -> not governed.
  int32_t cmd[4]   = { 1000, 1000, 1000, 1000 };
  float   meas[4]  = { 2100,    0, 2100, 2100 };
  float   out[4]   = { 1023, 1023, 1023, 1023 };
  bool    valid[4] = { true, false, true, true };
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, valid));
}

static void test_gov_uniform_ref_strong_half_not_dragged(void) {
  // Two battery halves: weak (FL,FR) max ~1600, strong (RL,RR) max ~2100. With the
  // UNIFORM target reference = weakest max (1600), cmd 1000 targets 1600 for ALL.
  // Weak wheels hit 1600 (their ceiling, saturated but ratio 1.0); strong wheels
  // hit 1600 easily at LOW pwm (not saturated). Nothing should drag the group:
  // the cart is already straight at the achievable common speed. This is the case
  // a per-wheel-max judge would get wrong (it would call everyone 1.0 too, but for
  // the wrong reason and would miss a strong wheel that later sags below 1600).
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   ref     = 1600.0f;
  float   meas[4] = { 1600, 1600, 1600, 1600 };   // all at the common target
  float   out[4]  = { 1010, 1010,  780,  780 };   // weak near max, strong relaxed
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedGovernorScale(cmd, meas, out, ref, GPWM, GLO, GSAT, 0));
}

static void test_gov_uniform_ref_sagging_wheel_drags(void) {
  // Same uniform ref 1600, but a weak wheel now SAGS to 1200 (0.75) and is pinned
  // at max PWM -> it defines the achievable group speed -> group scales to 0.75.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   ref     = 1600.0f;
  float   meas[4] = { 1200, 1600, 1600, 1600 };   // FL sags: 1200/1600 = 0.75
  float   out[4]  = { 1023,  780,  780,  780 };   // FL pinned at max
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.75f, speedGovernorScale(cmd, meas, out, ref, GPWM, GLO, GSAT, 0));
}

static void test_gov_throttled_wheel_does_not_latch(void) {
  // LATCH REGRESSION: the governor's scale is fed back to the commands, so when it
  // is already throttling, every wheel measures low (meas ~ scale*target) at LOW
  // pwm. The old rule judged these low-ratio wheels and stayed floored forever
  // (forward drive stuck at 10%). Now: all unsaturated -> none drag -> scale 1.0,
  // so the next tick raises drive and the loop climbs back out. Must be 1.0.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   ref     = 2100.0f;
  float   meas[4] = { 210, 210, 210, 210 };   // all at ~0.1 (throttled), ratio 0.1
  float   out[4]  = { 102, 102, 102, 102 };   // LOW pwm — not saturated
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedGovernorScale(cmd, meas, out, ref, GPWM, GLO, GSAT, 0));
}

static void test_gov_wrong_way_wheel_floors(void) {
  // A wheel turning OPPOSITE its command -> ratio forced 0 -> floor.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   meas[4] = { 2100, 2100, -500, 2100 };
  float   out[4]  = { 1023, 1023, 1023, 1023 };
  TEST_ASSERT_EQUAL_FLOAT(0.1f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

static void test_gov_uniform_load_no_spurious_throttle(void) {
  // BUG-001: ALL four wheels saturated (payload) and ALL sagging EQUALLY below the
  // no-load ref (ratio 0.6 each). The old absolute judge read "all four failing" and
  // floored the group, crawling loaded forward and forcing forward+turn to escape
  // the floor only by adding yaw (the spin). The group-relative judge sees worst==
  // best -> nothing to cross-correct -> NO throttle. This is the core forward+turn
  // fix: a uniformly loaded cart keeps its commanded speed and stays additive.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   meas[4] = { 1260, 1260, 1260, 1260 };   // 1260/2100 = 0.6 on every wheel
  float   out[4]  = { 1023, 1023, 1023, 1023 };   // all pinned at max under load
  TEST_ASSERT_EQUAL_FLOAT(1.0f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

static void test_gov_relative_lagging_corner_still_drags(void) {
  // BUG-001 must NOT blind the governor to a genuine asymmetric lag: three wheels
  // track at 0.9 while one loaded corner sags to 0.45 (half the others). The group
  // slows to the laggard's SHARE of the achievable best: 0.45/0.9 = 0.5. Straightness
  // protection is preserved exactly where it is needed.
  int32_t cmd[4]  = { 1000, 1000, 1000, 1000 };
  float   meas[4] = {  945, 1890, 1890, 1890 };   // FL 0.45, others 0.90
  float   out[4]  = { 1023, 1023, 1023, 1023 };   // all saturated
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, speedGovernorScale(cmd, meas, out, GMAX, GPWM, GLO, GSAT, 0));
}

// ---------------- governorRotationRelax (spin relax) ----------------
// Fade throttle depth by rotation fraction. relax=1 unless noted.
// transW (last arg) de-weights translation in the spin metric (BUG-002); 0.5 is
// the shipped value. Endpoints (pure spin / pure translate) are transW-independent.
#define GTW 0.5f
static void test_gov_relax_pure_spin_disables(void) {
  // Pure spin (vx=vy=0): a hard 0.1 throttle fully relaxes back to 1.0 so the
  // spin is not crawled by symmetric scrub load. transW irrelevant (at=0).
  TEST_ASSERT_EQUAL_FLOAT(1.0f, governorRotationRelax(0.1f, 0, 0, 1000, 1.0f, GTW));
}

static void test_gov_relax_pure_translate_unchanged(void) {
  // Pure translation (omega=0): governor keeps full authority, scale untouched.
  TEST_ASSERT_EQUAL_FLOAT(0.1f, governorRotationRelax(0.1f, 1000, 0, 0, 1.0f, GTW));
  TEST_ASSERT_EQUAL_FLOAT(0.4f, governorRotationRelax(0.4f, 0, 1000, 0, 1.0f, GTW));
}

static void test_gov_relax_diagonal_partial(void) {
  // Equal translate + rotate with transW=0.5: spin = 1000/(1000+0.5*1000)=0.667,
  // keep=1-0.667=0.333 -> 1-(1-0.5)*0.333 = 0.833.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8335f,
                           governorRotationRelax(0.5f, 1000, 0, 1000, 1.0f, GTW));
}

static void test_gov_relax_translation_dominant_gets_more_relief(void) {
  // BUG-002: forward + small turn under a deep throttle must be relieved MORE than
  // the old fraction metric (transW=1) gave, so adding a small turn does not leave
  // the forward part crawling. Old: relax(0.2,600,0,100,1,1.0)=0.314.
  float old_metric = governorRotationRelax(0.2f, 600, 0, 100, 1.0f, 1.0f);
  float fixed      = governorRotationRelax(0.2f, 600, 0, 100, 1.0f, GTW);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.314f, old_metric);   // documents the old value
  TEST_ASSERT_TRUE(fixed > old_metric + 0.05f);           // meaningfully more relief
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, fixed);          // spin=0.25 -> 0.4
}

static void test_gov_relax_disabled_passthrough(void) {
  // relax=0 -> feature off, scale passes through even on pure spin.
  TEST_ASSERT_EQUAL_FLOAT(0.1f, governorRotationRelax(0.1f, 0, 0, 1000, 0.0f, GTW));
}

static void test_gov_relax_idle_no_command(void) {
  // No command at all -> nothing to relax, scale returned as-is.
  TEST_ASSERT_EQUAL_FLOAT(1.0f, governorRotationRelax(1.0f, 0, 0, 0, 1.0f, GTW));
}

// ---------------- forwardKinematics (body-space estimate) ----------------
// Inverse of mecanumMix's linear core. Round-trip through the actual mix so the
// forward transform stays self-consistent with it even if a sign is later flipped.
#define FREF 2100.0f

static void fwd_roundtrip(int16_t vx_in, int16_t vy_in, int16_t w_in,
                          float* vx, float* vy, float* w, float* s) {
  int32_t c[4];
  mecanumMix(vx_in, vy_in, w_in, c);
  float meas[4];
  for (int i = 0; i < 4; i++) meas[i] = (float)c[i] / 1000.0f * FREF;  // perfect tracking
  forwardKinematics(meas, FREF, vx, vy, w, s);
}

static void test_fwd_pure_forward(void) {
  float vx, vy, w, s;
  fwd_roundtrip(1000, 0, 0, &vx, &vy, &w, &s);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, vx);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, vy);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, w);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, s);   // a commanded twist has zero slip
}

static void test_fwd_pure_strafe(void) {
  float vx, vy, w, s;
  fwd_roundtrip(0, 1000, 0, &vx, &vy, &w, &s);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, vx);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, vy);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, w);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, s);
}

static void test_fwd_pure_rotate(void) {
  float vx, vy, w, s;
  fwd_roundtrip(0, 0, 1000, &vx, &vy, &w, &s);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, vx);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, vy);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, w);
  // KEY: a normal rotation must report ZERO slip. The wrong null vector
  // [+1,-1,+1,-1] would read s=-1000 here and false-flag every spin.
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, s);
}

static void test_fwd_null_mode_is_slip(void) {
  // Hand-crafted null pattern [-1,-1,+1,+1] (front pair vs rear pair): no chassis
  // motion, pure slip. vx=vy=omega=0, s=full.
  float meas[4] = { -FREF, -FREF, FREF, FREF };
  float vx, vy, w, s;
  forwardKinematics(meas, FREF, &vx, &vy, &w, &s);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, vx);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, vy);
  TEST_ASSERT_FLOAT_WITHIN(1.0f,    0.0f, w);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, s);
}

static void test_fwd_uniform_ref_weak_wheel_reports_low(void) {
  // All four commanded forward but FL only reaches 0.8*ref -> recovered vx < 1000
  // (the curve a per-wheel-max normalization would hide). vy/omega pick up the
  // asymmetry; only the magnitude-below-1000 fact is locked here.
  float meas[4] = { 0.8f * FREF, FREF, FREF, FREF };
  float vx, vy, w, s;
  forwardKinematics(meas, FREF, &vx, &vy, &w, &s);
  TEST_ASSERT_TRUE(vx < 1000.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 950.0f, vx);  // (0.8+1+1+1)/4 * 1000
}

// ---------------- bodyCorrection (body-space outer loop) ----------------
static BodyLoopCfg body_cfg(void) {
  // kpTrans,kpW,kiTrans,kiW,iMax,wThresh,rateLimit,corrMax,yawFwdFrac,iDecay
  BodyLoopCfg c = { 0.15f, 0.28f, 0.10f, 0.15f, 150.0f, 150.0f, 400.0f, 200.0f,
                    0.5f, 3.0f };
  return c;
}
static float bl_abs(float v) { return v < 0 ? -v : v; }
static BodyLoopCfg body_cfg_ponly(void) {
  BodyLoopCfg c = body_cfg();
  c.kiTrans = 0.0f; c.kiW = 0.0f;
  return c;
}

// Drive the loop to steady state (rate limit + integral settle) at fixed inputs.
static void body_settle(BodyLoopState* st, const BodyLoopCfg* c,
                        float vxc, float vyc, float wc,
                        float vxm, float vym, float wm,
                        bool freeze, float gov,
                        float* dvx, float* dvy, float* dw) {
  // fwdAuth = throttled forward authority (|commanded translation| * govScale),
  // matching the drive-loop call site (BUG-004/005 relative yaw cap).
  float fwd = (bl_abs(vxc) + bl_abs(vyc)) * gov;
  for (int k = 0; k < 400; k++)
    bodyCorrection(vxc, vyc, wc, vxm, vym, wm, freeze, gov, fwd, 0.01f, c, st, dvx, dvy, dw);
}

static void test_body_zero_error_zero_corr(void) {
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg();
  float dvx, dvy, dw;
  body_settle(&st, &c, 300, 0, 0, 300, 0, 0, false, 1.0f, &dvx, &dvy, &dw);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dvx);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dvy);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dw);
}

static void test_body_yaw_during_translation(void) {
  // Translating (omega_cmd=0 < thresh) with an unwanted +100 measured yaw ->
  // opposing (negative) yaw correction, ~zero translation correction.
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg_ponly();
  float dvx, dvy, dw;
  body_settle(&st, &c, 500, 0, 0, 500, 0, 100, false, 1.0f, &dvx, &dvy, &dw);
  TEST_ASSERT_TRUE(dw < -1.0f);                         // resists the yaw
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dvx);          // translation weight ~0
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dvy);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -28.0f, dw);           // kpW * eW = 0.28 * -100
}

static void test_body_trans_during_rotation(void) {
  // Rotating (omega_cmd=500 > thresh) with unwanted +80 vx drift -> opposing vx
  // correction, ~zero yaw correction (yaw weight goes to 0 while rotating).
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg_ponly();
  float dvx, dvy, dw;
  body_settle(&st, &c, 0, 0, 500, 80, 0, 500, false, 1.0f, &dvx, &dvy, &dw);
  TEST_ASSERT_TRUE(dvx < -1.0f);                        // resists the drift
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dw);           // yaw weight ~0
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -12.0f, dvx);          // kpTrans * eVx = 0.15 * -80
}

static void test_body_saturation_freezes_integral(void) {
  // With freeze=true the integral never accumulates regardless of standing error.
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg();
  float dvx, dvy, dw;
  body_settle(&st, &c, 500, 0, 0, 500, 0, 100, true, 1.0f, &dvx, &dvy, &dw);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, st.iw);
  // Sanity: WITHOUT freeze the same error DOES wind the integral.
  BodyLoopState st2 = {};
  body_settle(&st2, &c, 500, 0, 0, 500, 0, 100, false, 1.0f, &dvx, &dvy, &dw);
  TEST_ASSERT_TRUE(st2.iw < -0.5f);
}

static void test_body_governor_scales_both_trans_and_yaw(void) {
  // BUG-001 fix: BOTH translation AND yaw corrections fade with govScale.
  // (Was: yaw kept full authority — that let an un-throttled yaw differential
  // dominate a governor-throttled forward command and turn a curve into a spin.)
  BodyLoopState a = {}, b = {};
  BodyLoopCfg c = body_cfg_ponly();
  float dvx_full, dvy, dw, dvx_half;
  // Rotating + vx drift: compare govScale 1.0 vs 0.5 -> trans correction halves.
  body_settle(&a, &c, 0, 0, 500, 80, 0, 500, false, 1.0f, &dvx_full, &dvy, &dw);
  body_settle(&b, &c, 0, 0, 500, 80, 0, 500, false, 0.5f, &dvx_half, &dvy, &dw);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.5f * dvx_full, dvx_half);
  // Translating + yaw error: govScale MUST now halve the yaw correction too, so
  // it can never out-authorize a throttled forward command.
  BodyLoopState y1 = {}, y2 = {};
  float dwf, dwh, jx, jy;
  body_settle(&y1, &c, 500, 0, 0, 500, 0, 100, false, 1.0f, &jx, &jy, &dwf);
  body_settle(&y2, &c, 500, 0, 0, 500, 0, 100, false, 0.5f, &jx, &jy, &dwh);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.5f * dwf, dwh);
}

static void test_body_yawhold_active_through_modest_turn(void) {
  // BUG-008: with the production BODY_W_THRESH, a modest commanded turn
  // (omega_cmd=300) must NOT have hard-switched yaw-hold off. A measured yaw
  // overshoot (w_m beyond w_c) must still draw an opposing yaw correction.
  // (Old thresh=150 gave rot=1 at omega=300 -> wYaw=0 -> dw=0; the bug.)
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg_ponly();
  c.wThresh = BODY_W_THRESH;                 // track the shipped config
  float dvx, dvy, dw;
  // commanded fwd 500 + turn 300; measured yaw overshoots to 400 -> eW=-100.
  body_settle(&st, &c, 500, 0, 300, 500, 0, 400, false, 1.0f, &dvx, &dvy, &dw);
  TEST_ASSERT_TRUE(dw < -1.0f);              // yaw-hold still acts through the turn
}

static void test_body_correction_rate_limited(void) {
  // One tick from rest with a huge error: output bounded by rateLimit*dt.
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg();
  float dvx, dvy, dw;
  bodyCorrection(0, 0, 0, 0, 0, 5000, false, 1.0f, 0.0f, 0.01f, &c, &st, &dvx, &dvy, &dw);
  float adw = dw < 0 ? -dw : dw;
  TEST_ASSERT_TRUE(adw <= c.rateLimit * 0.01f + 0.001f);   // <= 4 cmd units
}

static void test_body_curve_under_uniform_sag_no_phantom_yaw(void) {
  // BUG-003: forward+turn (vx 600, omega 200). Under payload the chassis curves
  // CORRECTLY but slower — both vx and omega measure ~0.7x. The raw error w_c - w_m
  // = 200 - 140 = +60 would read as "under-rotating" and inject MORE yaw (the spin).
  // The track-ratio reference (w_c * vx_m/vx_c = 200*0.7 = 140) makes eW ≈ 0, so no
  // phantom yaw is added: the curve is left alone.
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg_ponly();
  c.wThresh = BODY_W_THRESH;
  float dvx, dvy, dw;
  body_settle(&st, &c, 600, 0, 200, 420, 0, 140, false, 1.0f, &dvx, &dvy, &dw);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, dw);   // no injected yaw from uniform sag
}

static void test_body_yaw_clamped_relative_to_throttled_forward(void) {
  // BUG-004/005: translating with a real yaw drift (w_m=300) but a SMALL throttled
  // forward authority (fwdAuth=100). corrW would be 0.28*300=84 (well under the
  // absolute ±200 clamp), but the relative cap = yawFwdFrac*fwdAuth = 0.5*100 = 50
  // bounds it so yaw can never dominate a shrunken forward base. With a LARGE
  // fwdAuth the absolute clamp governs and the full 84 survives.
  BodyLoopCfg c = body_cfg_ponly();
  c.wThresh = BODY_W_THRESH;
  float dvx, dvy, dw;
  BodyLoopState st1 = {};
  for (int k = 0; k < 400; k++)
    bodyCorrection(600, 0, 0, 600, 0, 300, false, 1.0f, 100.0f, 0.01f, &c, &st1, &dvx, &dvy, &dw);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -50.0f, dw);   // capped to 0.5 * fwdAuth
  BodyLoopState st2 = {};
  for (int k = 0; k < 400; k++)
    bodyCorrection(600, 0, 0, 600, 0, 300, false, 1.0f, 1000.0f, 0.01f, &c, &st2, &dvx, &dvy, &dw);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -84.0f, dw);   // generous fwdAuth -> P term survives
}

static void test_body_freeze_drops_proportional_yaw(void) {
  // BUG-006: when the governor owns magnitude (freeze=true) the body loop must yield
  // HEADING too — the proportional yaw term stands down, not just the integral. With
  // a standing yaw error the frozen correction is ~0; unfrozen it is the full P push.
  BodyLoopState fr = {}, live = {};
  BodyLoopCfg c = body_cfg_ponly();
  float dvx, dvy, dw_frozen, dw_live;
  body_settle(&fr,   &c, 500, 0, 0, 500, 0, 100, true,  1.0f, &dvx, &dvy, &dw_frozen);
  body_settle(&live, &c, 500, 0, 0, 500, 0, 100, false, 1.0f, &dvx, &dvy, &dw_live);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, dw_frozen);   // P gated under freeze
  TEST_ASSERT_TRUE(dw_live < -1.0f);                 // P acts off-freeze
}

static void test_body_frozen_integral_decays(void) {
  // BUG-007: an integral wound up while unfrozen must BLEED toward zero while frozen,
  // not HOLD and dump as a heading kick when the freeze threshold is recrossed.
  BodyLoopState st = {};
  BodyLoopCfg c = body_cfg();                 // with KI so the integral can wind
  c.wThresh = BODY_W_THRESH;
  float dvx, dvy, dw;
  body_settle(&st, &c, 500, 0, 0, 500, 0, 100, false, 1.0f, &dvx, &dvy, &dw);
  float wound = st.iw;
  TEST_ASSERT_TRUE(bl_abs(wound) > 1.0f);     // it actually wound up
  for (int k = 0; k < 200; k++)               // now hold it frozen
    bodyCorrection(500, 0, 0, 500, 0, 100, true, 1.0f, 500.0f, 0.01f, &c, &st, &dvx, &dvy, &dw);
  TEST_ASSERT_TRUE(bl_abs(st.iw) < bl_abs(wound) * 0.5f);   // decayed substantially
}

// ---------------- applyCurve ----------------

static void test_curve_zero_and_deadzone(void) {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applyCurve(0, CURVE));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applyCurve(10, CURVE));   // x=0.01 < deadzone 0.05
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applyCurve(-10, CURVE));
}

static void test_curve_sign_preserved(void) {
  TEST_ASSERT_TRUE(applyCurve(500, CURVE)  > 0.0f);
  TEST_ASSERT_TRUE(applyCurve(-500, CURVE) < 0.0f);
}

static void test_curve_monotonic(void) {
  TEST_ASSERT_TRUE(applyCurve(800, CURVE) > applyCurve(400, CURVE));
  TEST_ASSERT_TRUE(applyCurve(400, CURVE) > applyCurve(200, CURVE));
}

static void test_curve_bounded(void) {
  float full = applyCurve(1000, CURVE);
  TEST_ASSERT_TRUE(full <= 1.0001f && full >= 0.99f);
  TEST_ASSERT_TRUE(applyCurve(-1000, CURVE) >= -1.0001f);
}

// ---------------- CURVE_OMEGA (turn axis, BUG-010/011) ----------------

static void test_curve_omega_responsive_small_turn(void) {
  // BUG-010: a slight turn must produce meaningful commanded yaw. CURVE_OMEGA must
  // give a small input MUCH more output than the logistic CURVE (which crushes it),
  // and have no redundant second deadzone at low deflection.
  float small_omega = applyCurve(100, CURVE_OMEGA);
  float small_curve = applyCurve(100, CURVE);
  TEST_ASSERT_TRUE(small_omega > 0.0f);
  TEST_ASSERT_TRUE(small_omega > 3.0f * small_curve);   // far more responsive
}

static void test_curve_omega_monotonic_bounded_signed(void) {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, applyCurve(0, CURVE_OMEGA));
  TEST_ASSERT_TRUE(applyCurve(800, CURVE_OMEGA) > applyCurve(400, CURVE_OMEGA));
  TEST_ASSERT_TRUE(applyCurve(400, CURVE_OMEGA) > applyCurve(100, CURVE_OMEGA));
  TEST_ASSERT_TRUE(applyCurve(-500, CURVE_OMEGA) < 0.0f);
  float full = applyCurve(1000, CURVE_OMEGA);
  TEST_ASSERT_TRUE(full <= 1.0001f && full >= 0.99f);
  TEST_ASSERT_TRUE(applyCurve(-1000, CURVE_OMEGA) >= -1.0001f);
}

// ---------------- normalize ----------------

static void test_normalize_center_zero(void) {
  TEST_ASSERT_EQUAL_INT16(0, normalize(2048, 2048, DEADZONE_RAW, HALF_RANGE));
}

static void test_normalize_deadband_zero(void) {
  TEST_ASSERT_EQUAL_INT16(0, normalize(2048 + 50, 2048, DEADZONE_RAW, HALF_RANGE));
  TEST_ASSERT_EQUAL_INT16(0, normalize(2048 - 50, 2048, DEADZONE_RAW, HALF_RANGE));
}

static void test_normalize_full_deflection(void) {
  TEST_ASSERT_EQUAL_INT16( 1000, normalize(2048 + HALF_RANGE, 2048, DEADZONE_RAW, HALF_RANGE));
  TEST_ASSERT_EQUAL_INT16(-1000, normalize(2048 - HALF_RANGE, 2048, DEADZONE_RAW, HALF_RANGE));
}

static void test_normalize_clamps(void) {
  TEST_ASSERT_EQUAL_INT16( 1000, normalize(2048 + 5000, 2048, DEADZONE_RAW, HALF_RANGE));
  TEST_ASSERT_EQUAL_INT16(-1000, normalize(0, 4000, DEADZONE_RAW, HALF_RANGE));
}

// ---------------- crc8 ----------------

static void test_crc8_known_vector(void) {
  const uint8_t v[9] = { '1','2','3','4','5','6','7','8','9' };
  TEST_ASSERT_EQUAL_HEX8(0xF4, crc8(v, 9));  // CRC-8/SMBUS check value
}

static void test_crc8_detects_corruption(void) {
  uint8_t buf[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  uint8_t good = crc8(buf, 8);
  buf[3] ^= 0x01;  // flip one bit
  TEST_ASSERT_NOT_EQUAL(good, crc8(buf, 8));
}

// ---------------- protocol ----------------

static void test_packet_wire_size(void) {
  // Robot drops any frame whose length != sizeof(CtrlPacket); locking the size
  // here catches accidental layout/alignment drift before it reaches hardware.
  TEST_ASSERT_EQUAL_UINT32(13, (uint32_t)sizeof(CtrlPacket));
}

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_mix_forward_all_equal);
  RUN_TEST(test_mix_strafe_diagonal_opposite);
  RUN_TEST(test_mix_spin_left_right_opposite);
  RUN_TEST(test_mix_saturation_caps_at_1000);
  RUN_TEST(test_normalize_quad_scales_over_limit);
  RUN_TEST(test_normalize_quad_under_limit_untouched);
  RUN_TEST(test_gov_all_tracking_no_scale);
  RUN_TEST(test_gov_held_wheel_pulls_group_to_floor);
  RUN_TEST(test_gov_forward_reverse_symmetric_held_wheel);
  RUN_TEST(test_gov_saturated_phantom_still_engages);
  RUN_TEST(test_gov_mild_lag_unsaturated_ignored);
  RUN_TEST(test_gov_zero_command_no_demand);
  RUN_TEST(test_gov_invalid_wheel_skipped);
  RUN_TEST(test_gov_uniform_ref_strong_half_not_dragged);
  RUN_TEST(test_gov_uniform_ref_sagging_wheel_drags);
  RUN_TEST(test_gov_throttled_wheel_does_not_latch);
  RUN_TEST(test_gov_wrong_way_wheel_floors);
  RUN_TEST(test_gov_uniform_load_no_spurious_throttle);
  RUN_TEST(test_gov_relative_lagging_corner_still_drags);
  RUN_TEST(test_gov_relax_pure_spin_disables);
  RUN_TEST(test_gov_relax_pure_translate_unchanged);
  RUN_TEST(test_gov_relax_diagonal_partial);
  RUN_TEST(test_gov_relax_translation_dominant_gets_more_relief);
  RUN_TEST(test_gov_relax_disabled_passthrough);
  RUN_TEST(test_gov_relax_idle_no_command);
  RUN_TEST(test_fwd_pure_forward);
  RUN_TEST(test_fwd_pure_strafe);
  RUN_TEST(test_fwd_pure_rotate);
  RUN_TEST(test_fwd_null_mode_is_slip);
  RUN_TEST(test_fwd_uniform_ref_weak_wheel_reports_low);
  RUN_TEST(test_body_zero_error_zero_corr);
  RUN_TEST(test_body_yaw_during_translation);
  RUN_TEST(test_body_trans_during_rotation);
  RUN_TEST(test_body_saturation_freezes_integral);
  RUN_TEST(test_body_governor_scales_both_trans_and_yaw);
  RUN_TEST(test_body_yawhold_active_through_modest_turn);
  RUN_TEST(test_body_correction_rate_limited);
  RUN_TEST(test_body_curve_under_uniform_sag_no_phantom_yaw);
  RUN_TEST(test_body_yaw_clamped_relative_to_throttled_forward);
  RUN_TEST(test_body_freeze_drops_proportional_yaw);
  RUN_TEST(test_body_frozen_integral_decays);
  RUN_TEST(test_curve_zero_and_deadzone);
  RUN_TEST(test_curve_sign_preserved);
  RUN_TEST(test_curve_monotonic);
  RUN_TEST(test_curve_bounded);
  RUN_TEST(test_curve_omega_responsive_small_turn);
  RUN_TEST(test_curve_omega_monotonic_bounded_signed);
  RUN_TEST(test_normalize_center_zero);
  RUN_TEST(test_normalize_deadband_zero);
  RUN_TEST(test_normalize_full_deflection);
  RUN_TEST(test_normalize_clamps);
  RUN_TEST(test_crc8_known_vector);
  RUN_TEST(test_crc8_detects_corruption);
  RUN_TEST(test_packet_wire_size);
  return UNITY_END();
}
