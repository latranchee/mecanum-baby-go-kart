// Host (native) unit tests for the pure logic headers. Run with: pio test -e native
// These lock the sign/remap/scaling math that has repeatedly needed fixing.
#include <unity.h>
#include <string.h>
#include "kinematics.h"
#include "control_math.h"
#include "config_controller.h"  // CURVE, DEADZONE_RAW, HALF_RANGE

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

void setUp(void) {}
void tearDown(void) {}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_mix_forward_all_equal);
  RUN_TEST(test_mix_strafe_diagonal_opposite);
  RUN_TEST(test_mix_spin_left_right_opposite);
  RUN_TEST(test_mix_saturation_caps_at_1000);
  RUN_TEST(test_curve_zero_and_deadzone);
  RUN_TEST(test_curve_sign_preserved);
  RUN_TEST(test_curve_monotonic);
  RUN_TEST(test_curve_bounded);
  RUN_TEST(test_normalize_center_zero);
  RUN_TEST(test_normalize_deadband_zero);
  RUN_TEST(test_normalize_full_deflection);
  RUN_TEST(test_normalize_clamps);
  RUN_TEST(test_crc8_known_vector);
  RUN_TEST(test_crc8_detects_corruption);
  return UNITY_END();
}
