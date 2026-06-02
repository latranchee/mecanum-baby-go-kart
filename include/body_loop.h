#pragma once
#include <stdint.h>

// Body-space outer loop (motion-correct mecanum compensation).
//
// The inner control is 4 INDEPENDENT per-wheel PI loops + one scalar governor.
// A per-wheel SISO loop physically cannot distinguish "this wheel is lagging"
// from "the whole chassis is yawing" — both look like a single tracking error to
// it. And the governor (the mix being linear) can only throttle the WHOLE twist
// uniformly: it preserves commanded direction but cannot actively HOLD heading
// against a disturbance. So the firmware has no concept of chassis-level intent.
//
// This loop closes that gap. It runs ON TOP of the inner loops in body space
// (vx,vy,omega), estimated from the wheels via forwardKinematics(), and emits a
// small additive twist correction. The per-motion compensation the design calls
// for IS the continuous weighting here:
//   - translating (|omega_cmd| small): weight YAW error high  -> actively hold heading
//   - rotating   (|omega_cmd| large): weight TRANS error high -> actively hold centre
// expressed as a smoothstep blend, so it IS the per-motion mode selector, not a
// discrete switch.
//
// HANDOFF INVARIANT with the governor (governor.h): the governor owns MAGNITUDE
// under saturation, the body loop owns HEADING under non-saturation. Made
// structural here: the outer integral is FROZEN whenever a wheel is saturated /
// the governor is throttling / slip is high; and when the governor is active the
// TRANSLATION correction is faded by govScale while the YAW correction stays at
// full authority (so "block a corner while driving forward" still gets heading
// actively resisted even as the governor throttles speed).
//
// Pure float math, no Arduino deps — host-testable (test/test_math).

struct BodyLoopState {
  float ix, iy, iw;      // per-axis integral accumulators (cmd units)
  float dvx, dvy, dw;    // last applied correction (rate-limit memory)
};

// Field order matches the aggregate initializer used at the call site.
struct BodyLoopCfg {
  float kpTrans, kpW;    // proportional gains (translation, yaw)
  float kiTrans, kiW;    // integral gains, per second (translation, yaw)
  float iMax;            // integral clamp, cmd units (~15% authority backstop)
  float wThresh;         // |omega_cmd| (cmd units) above which motion is "rotating"
  float rateLimit;       // correction slew, cmd units/sec
  float corrMax;         // hard per-axis correction clamp, cmd units
};

static inline float bl_clamp(float v, float lim) {
  if (v >  lim) return  lim;
  if (v < -lim) return -lim;
  return v;
}

// Hermite smoothstep, clamped to [0,1].
static inline float bl_smoothstep(float edge0, float edge1, float x) {
  if (edge1 <= edge0) return x < edge0 ? 0.0f : 1.0f;
  float t = (x - edge0) / (edge1 - edge0);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

static inline float bl_rate(float cur, float tgt, float maxStep) {
  float d = tgt - cur;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  return cur + d;
}

//   vx_c,vy_c,w_c  : commanded body twist (cmd units, the packet's vx/vy/omega).
//   vx_m,vy_m,w_m  : measured (IIR-filtered) body twist from forwardKinematics().
//   freezeIntegral : true => do not integrate this tick (saturation/governor/slip).
//   govScale       : current governor scale [floor..1]; fades TRANSLATION corr only.
//   dt             : seconds since last call.
//   cfg, st        : config + persistent state (caller owns lifetime).
//   out dvx,dvy,dw : rate-limited additive twist correction (cmd units).
static inline void bodyCorrection(
    float vx_c, float vy_c, float w_c,
    float vx_m, float vy_m, float w_m,
    bool freezeIntegral, float govScale, float dt,
    const BodyLoopCfg* cfg, BodyLoopState* st,
    float* dvx, float* dvy, float* dw) {

  // Continuous per-motion weight: rot~1 when rotating, ~0 when translating.
  float aw     = w_c < 0 ? -w_c : w_c;
  float rot    = bl_smoothstep(0.0f, cfg->wThresh, aw);
  float wYaw   = 1.0f - rot;   // yaw-hold dominates when translating
  float wTrans = rot;          // centre-hold dominates when rotating

  float eVx = vx_c - vx_m;
  float eVy = vy_c - vy_m;
  float eW  = w_c  - w_m;

  // Integrate only when the governor is NOT in charge (heading authority), and
  // weight each axis the same way its proportional term is weighted so an axis
  // that is currently "off" (e.g. translation while driving straight) does not
  // silently wind up.
  if (!freezeIntegral) {
    st->iw += eW  * cfg->kiW     * dt * wYaw;
    st->ix += eVx * cfg->kiTrans * dt * wTrans;
    st->iy += eVy * cfg->kiTrans * dt * wTrans;
  }
  // Clamp unconditionally so a lowered iMax always bounds (backstop authority).
  st->iw = bl_clamp(st->iw, cfg->iMax);
  st->ix = bl_clamp(st->ix, cfg->iMax);
  st->iy = bl_clamp(st->iy, cfg->iMax);

  float corrW  = wYaw   * (cfg->kpW     * eW  + st->iw);
  float corrVx = wTrans * (cfg->kpTrans * eVx + st->ix);
  float corrVy = wTrans * (cfg->kpTrans * eVy + st->iy);

  // Governor handoff: it throttles MAGNITUDE, so fade translation correction with
  // govScale — but keep YAW correction at full authority so heading is still
  // actively resisted while a blocked corner is being throttled.
  corrVx *= govScale;
  corrVy *= govScale;

  corrW  = bl_clamp(corrW,  cfg->corrMax);
  corrVx = bl_clamp(corrVx, cfg->corrMax);
  corrVy = bl_clamp(corrVy, cfg->corrMax);

  // Rate-limit toward the target (anti-jerk; the correction is near-zero net
  // current so it needs no inrush ramp — only a slew on its own change).
  float maxStep = cfg->rateLimit * dt;
  st->dw  = bl_rate(st->dw,  corrW,  maxStep);
  st->dvx = bl_rate(st->dvx, corrVx, maxStep);
  st->dvy = bl_rate(st->dvy, corrVy, maxStep);

  *dw  = st->dw;
  *dvx = st->dvx;
  *dvy = st->dvy;
}
