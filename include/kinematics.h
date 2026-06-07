#pragma once
#include <stdint.h>

// Mecanum wheel mixing — pure integer math, no Arduino deps (host-testable).
// Wheel command in [-1000..+1000] (thousandths of max wheel speed).
// X-pattern rollers, slot map: M1=FL, M2=FR, M3=RL, M4=RR.
//
// STRAFE-SIGN CAVEAT (audit 2026-06-02, unresolved on hardware):
//   protocol.h declares vy = strafe-RIGHT(+) and the controller maps stick-right
//   -> +vy, but the matrix below is the textbook vy=+LEFT form: pure vy>0 gives
//   wheel signs {FL-, FR+, RL+, RR-}, which is the strafe-LEFT pattern. So by the
//   book, commanding "strafe right" would translate the robot LEFT.
//   NOT changed here: test_mix_strafe_diagonal_opposite locks this exact mapping
//   and no strafe fault has been reported, so the physical roller handedness on
//   this build may already make {FL-,FR+,RL+,RR-} = right. Forward (all +) and
//   rotate are unaffected either way.
//   To verify on the floor: in robot test mode send `t 0 500 0` (vy=+500). If the
//   robot strafes LEFT, the sign is wrong — flip the vy terms below (FL+vy, FR-vy,
//   RL-vy, RR+vy) and update the test. If it strafes RIGHT, leave as-is.
// Scale a 4-wheel command set so its peak magnitude does not exceed `limit`,
// preserving the ratio between wheels (direction unchanged). `limit` must be > 0.
// Used by mecanumMix AND by the drive loop to re-normalize the governor-scaled
// base PLUS the body-loop correction as ONE valid mecanum set (BUG-007): without
// it, base+correction could push a single wheel past ±1000 into pidStep, pinning
// one wheel while peers do not — reintroducing the cross-wheel yaw the outer
// loops exist to remove. Pure integer math, host-testable.
static inline void normalizeQuad(int32_t c[4], int32_t limit) {
  int32_t peak = limit;
  for (int i = 0; i < 4; i++) {
    int32_t a = c[i] < 0 ? -c[i] : c[i];
    if (a > peak) peak = a;
  }
  if (peak > limit) {
    for (int i = 0; i < 4; i++) c[i] = (c[i] * limit) / peak;
  }
}

static inline void mecanumMix(int16_t vx, int16_t vy, int16_t omega, int32_t outCmd[4]) {
  outCmd[0] = (int32_t)vx - vy - omega;  // FL
  outCmd[1] = (int32_t)vx + vy + omega;  // FR
  outCmd[2] = (int32_t)vx + vy - omega;  // RL
  outCmd[3] = (int32_t)vx - vy + omega;  // RR

  normalizeQuad(outCmd, 1000);
}

// Forward kinematics — recover the body twist (vx,vy,omega) and a slip/null
// coordinate (s) from measured wheel speeds. Exact inverse of mecanumMix's linear
// core: its three input columns are mutually orthogonal with norm^2 = 4, so each
// body axis = (its column . wheels)/4. Pure integer/float math, host-testable.
//
//   measTps[i] : measured signed ticks/sec per wheel, slot order [FL,FR,RL,RR]
//                (the encSign-corrected, glitch-clamped lastMeasTps[]).
//   refTps     : UNIFORM normalization reference (= maxTpsMin()) — the SAME scalar
//                the inner loop targets for cmd 1000. Dividing by it puts the body
//                estimate back in cmd units (+/-1000 == refTps). MUST NOT be the
//                per-wheel MAX_TPS[i]: normalizing each wheel to its own max would
//                let a weak wheel report full speed and hide the very curve the
//                governor exists to catch (see governor.h uniform-ref note).
//   out vx,vy,omega : body twist in cmd units.
//   out s      : NULL/SLIP coordinate — the 4th axis orthogonal to all three motion
//                columns. mecanumMix can NEVER command it, so any nonzero s means
//                the wheels are fighting (a held corner, traction scrub) or an
//                encoder is faulting. OBSERVE / gate only — never actuated.
//
// NULL VECTOR for THIS mix is [-1,-1,+1,+1] (front pair vs rear pair), derived as
// the orthogonal complement of {vx,vy,omega} columns. (It is NOT [+1,-1,+1,-1] —
// that vector equals -omega, i.e. it would read nonzero on every normal rotation.)
static inline void forwardKinematics(const float measTps[4], float refTps,
                                     float* vx, float* vy, float* omega, float* s) {
  if (refTps <= 0.0f) { *vx = *vy = *omega = *s = 0.0f; return; }
  const float k = 1000.0f / refTps;             // ticks/sec -> cmd units
  float c0 = measTps[0] * k;  // FL
  float c1 = measTps[1] * k;  // FR
  float c2 = measTps[2] * k;  // RL
  float c3 = measTps[3] * k;  // RR
  *vx    = ( c0 + c1 + c2 + c3) * 0.25f;
  *vy    = (-c0 + c1 + c2 - c3) * 0.25f;
  *omega = (-c0 + c1 - c2 + c3) * 0.25f;
  *s     = (-c0 - c1 + c2 + c3) * 0.25f;        // [-1,-1,+1,+1] null axis
}
