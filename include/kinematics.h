#pragma once
#include <stdint.h>

// Mecanum wheel mixing — pure integer math, no Arduino deps (host-testable).
// Wheel command in [-1000..+1000] (thousandths of max wheel speed).
// X-pattern rollers, slot map: M1=FL, M2=FR, M3=RL, M4=RR.
//
// FRAME NOTE (2026-10-05): every name here (FL/FR/RL/RR, vx forward, vy right) is
// the FIRMWARE frame. The rider faces the firmware's rear: the controller sends
// vx and vy negated (config_controller.h INVERT_VX/VY), a 180-degree turn, and
// slot 0 ("FL") was bench-confirmed as the rider's rear-right. The rider's stick
// directions are correct in use; the old 2026-06-02 strafe-sign caveat (textbook
// vy=+LEFT matrix vs. "vy strafe-right+") is moot from the rider's side, because
// the controller's inversion is what was tuned against the real cart. Do not flip
// signs here without re-checking the controller inversions with it.
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

// Inverse of mecanumMix's linear core on a wheel COMMAND set: the body twist the
// four commands actually express, in cmd units. It differs from the packet twist
// whenever the mix normalized (|vx|+|vy|+|omega| > 1000) or the slew is mid-ramp,
// so any loop judging "what was asked" (governor spin relax, body loop) must use
// this, not the raw packet. Integer twin of forwardKinematics below.
static inline void mixInverse(const int32_t c[4], int32_t* vx, int32_t* vy, int32_t* omega) {
  *vx    = ( c[0] + c[1] + c[2] + c[3]) / 4;
  *vy    = (-c[0] + c[1] + c[2] - c[3]) / 4;
  *omega = (-c[0] + c[1] - c[2] + c[3]) / 4;
}

// Step a wheel command set toward its target by at most maxStep on any wheel,
// moving all four along the straight line between them so they arrive on the same
// tick. Clamping each wheel independently let small targets arrive first and bent
// the twist mid-ramp: forward+turn from rest drove straight for ~300 ms before the
// turn came in, and a release veered the other way. mecanumMix is linear, so a
// straight line in wheel space is a straight line in twist space and the commanded
// direction holds through the whole ramp. The inrush cap is unchanged: the wheel
// with the largest change steps exactly maxStep, every other wheel steps less.
// maxStep must be >= 1. Pure integer math (|d|*maxStep stays far inside int32).
static inline void slewQuad(int32_t cur[4], const int32_t tgt[4], int32_t maxStep) {
  int32_t big = 0;
  for (int i = 0; i < 4; i++) {
    int32_t a = tgt[i] - cur[i];
    if (a < 0) a = -a;
    if (a > big) big = a;
  }
  if (big <= maxStep) {
    for (int i = 0; i < 4; i++) cur[i] = tgt[i];
    return;
  }
  for (int i = 0; i < 4; i++) {
    int32_t num = (tgt[i] - cur[i]) * maxStep;
    cur[i] += (num >= 0 ? num + big / 2 : num - big / 2) / big;  // round half away
  }
}

// Forward kinematics — recover the body twist (vx,vy,omega) and a slip/null
// coordinate (s) from measured wheel speeds. Exact inverse of mecanumMix's linear
// core: its three input columns are mutually orthogonal with norm^2 = 4, so each
// body axis = (its column . wheels)/4. Pure integer/float math, host-testable.
//
//   measTps[i] : measured signed ticks/sec per wheel, slot order [FL,FR,RL,RR]
//                (the encSign-corrected, glitch-clamped lastMeasTps[]).
//   refTps     : UNIFORM normalization reference (= cmdRefTps()) — the SAME scalar
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
