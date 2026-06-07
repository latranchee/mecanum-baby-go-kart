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
// the governor is throttling / slip is high; and when the governor is active ALL
// corrections (translation AND yaw) are faded by govScale, so the body loop can
// never out-authorize a governor-throttled base command (BUG-001 — a full-
// authority yaw correction on a throttled forward command turned curves into
// spins). The governor's group-slowing is the primary heading protector when a
// single corner is blocked; the body loop refines heading only within the
// authority the governor currently allows.
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
  float yawFwdFrac;      // (BUG-004/005) yaw corr cap as a fraction of throttled fwd
  float iDecay;          // (BUG-007) frozen-integral bleed rate, 1/s
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
//   freezeIntegral : true => governor owns magnitude (saturation/throttle/slip):
//                    decay the integral AND drop the proportional term (BUG-006/007).
//   govScale       : current governor scale [floor..1]; fades all corrections.
//   fwdAuth        : current THROTTLED forward authority magnitude (cmd units,
//                    = |commanded translation| * govScale). Bounds the yaw
//                    correction relative to forward while translating (BUG-004/005).
//   dt             : seconds since last call.
//   cfg, st        : config + persistent state (caller owns lifetime).
//   out dvx,dvy,dw : rate-limited additive twist correction (cmd units).
static inline void bodyCorrection(
    float vx_c, float vy_c, float w_c,
    float vx_m, float vy_m, float w_m,
    bool freezeIntegral, float govScale, float fwdAuth, float dt,
    const BodyLoopCfg* cfg, BodyLoopState* st,
    float* dvx, float* dvy, float* dw) {

  // Continuous per-motion weight: rot~1 when rotating, ~0 when translating.
  float aw     = w_c < 0 ? -w_c : w_c;
  float rot    = bl_smoothstep(0.0f, cfg->wThresh, aw);
  float wYaw   = 1.0f - rot;   // yaw-hold dominates when translating
  float wTrans = rot;          // centre-hold dominates when rotating

  // (BUG-003) Track-ratio yaw reference. forwardKinematics has NO load model: on a
  // curve the deliberately-faster wheels sag more under payload, so the recovered
  // w_m drops below the commanded w_c even when the chassis is curving CORRECTLY
  // (just slower) — vx_m sags by the same fraction. Chasing the raw w_c then forces
  // yaw up while translation stays throttled = curve collapses into a spin. Scale
  // the yaw reference by the fraction of commanded translation actually achieved:
  // under uniform sag w_c*trackRatio ≈ w_m so eW≈0 (no phantom yaw), while a REAL
  // unwanted yaw (translation tracking fine, chassis drifting) still shows full eW.
  float transCmd  = (vx_c < 0 ? -vx_c : vx_c) + (vy_c < 0 ? -vy_c : vy_c);
  float transMeas = (vx_m < 0 ? -vx_m : vx_m) + (vy_m < 0 ? -vy_m : vy_m);
  float trackRatio = 1.0f;
  if (transCmd > 1.0f) {
    trackRatio = transMeas / transCmd;
    if (trackRatio > 1.0f) trackRatio = 1.0f;
    if (trackRatio < 0.0f) trackRatio = 0.0f;
  }
  float wRef = w_c * trackRatio;

  float eVx = vx_c - vx_m;
  float eVy = vy_c - vy_m;
  float eW  = wRef - w_m;

  // Integral: when the governor owns magnitude, DECAY toward zero (BUG-007) so a
  // value wound up in an unfrozen window cannot HOLD across the freeze threshold
  // and dump as a heading kick. Otherwise integrate, weighting each axis like its
  // proportional term so an "off" axis does not silently wind up.
  if (freezeIntegral) {
    float dk = cfg->iDecay * dt;
    if (dk > 1.0f) dk = 1.0f;
    st->iw -= st->iw * dk;
    st->ix -= st->ix * dk;
    st->iy -= st->iy * dk;
  } else {
    st->iw += eW  * cfg->kiW     * dt * wYaw;
    st->ix += eVx * cfg->kiTrans * dt * wTrans;
    st->iy += eVy * cfg->kiTrans * dt * wTrans;
  }
  // Clamp unconditionally so a lowered iMax always bounds (backstop authority).
  st->iw = bl_clamp(st->iw, cfg->iMax);
  st->ix = bl_clamp(st->ix, cfg->iMax);
  st->iy = bl_clamp(st->iy, cfg->iMax);

  // (BUG-006) Governor handoff is structural: when frozen the governor owns the
  // situation, so the body loop yields HEADING too — drop the proportional term
  // (which would otherwise keep firing on the standing error the governor is
  // already handling). Only the decaying integral remains. Off-freeze, full P.
  float pGate = freezeIntegral ? 0.0f : 1.0f;
  float corrW  = wYaw   * (cfg->kpW     * eW  * pGate + st->iw);
  float corrVx = wTrans * (cfg->kpTrans * eVx * pGate + st->ix);
  float corrVy = wTrans * (cfg->kpTrans * eVy * pGate + st->iy);

  // Fade all corrections by govScale (absolute-jerk bound). NOTE this alone does
  // NOT bound the curve-vs-spin RATIO — base and correction both scale by govScale
  // so it cancels in yaw:forward (BUG-004). The relative cap below is what bounds it.
  corrW  *= govScale;
  corrVx *= govScale;
  corrVy *= govScale;

  // (BUG-004/005) While translating, cap the yaw correction to a fraction of the
  // CURRENT throttled forward authority, not an absolute ±corrMax. So however far
  // the governor has shrunk forward, yaw stays a minority of it and a curve can
  // never become a spin. When not translating (fwdAuth≈0) the yaw-hold weight is
  // ~0 anyway; fall back to the absolute clamp.
  float capW = cfg->corrMax;
  if (fwdAuth > 1.0f) {
    float rel = cfg->yawFwdFrac * fwdAuth;
    if (rel < capW) capW = rel;
  }
  corrW  = bl_clamp(corrW,  capW);
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
