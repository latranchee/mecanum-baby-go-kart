#pragma once
#include <stdint.h>

// Electronic-differential speed governor (cross-wheel synchronization).
//
// The per-wheel PI loops in pidStep() are INDEPENDENT: each tracks its own
// target and never sees the others. So holding/stalling one wheel does not slow
// the rest — the cart yaws about the held corner while the free wheels keep
// their full speed. mecanumMix() only produces open-loop targets; no coupling.
//
// This computes ONE group scale in [loScale..1.0] from the WORST-tracking driven
// wheel. The caller multiplies every wheel's command by it, so when one wheel
// can't keep up (held, stalled, traction- or supply-limited) the others slow to
// match and the cart stays pointed straight instead of spinning.
//
// SIGN-INDEPENDENT BY DESIGN (audit 2026-06-02 round 2): an earlier version used
// a signed ratio meas/tgt. That looked symmetric, but a held-yet-energized wheel
// emits direction-asymmetric PHANTOM encoder counts (1x ISR decode), so in
// FORWARD the phantom read as "keeping up" while REVERSE engaged by luck. The fix:
// judge by MAGNITUDE |meas|/|tgt| (sign-independent). [1x decode replaced by 4x
// quadrature 2026-06-02, so phantoms are largely gone, but the magnitude judge stays.]
//
// ONLY SATURATED WHEELS LIMIT THE GROUP (latch fix 2026-06-02 round 3): the scale
// this returns is applied back to the commands, so a wheel commanded 0.1x only
// MEASURES 0.1x — judging an UNSATURATED wheel by its ratio made the governor read
// its own throttling as wheel-incapacity and LATCH at the floor (forward drive
// stuck at 10%, never recovering, since meas always equals govScale*target). Cure:
// a wheel can only define the achievable group speed if it is SATURATED (pinned
// near max PWM yet still below target) — it physically cannot deliver more. An
// unsaturated wheel has PWM headroom (or is merely throttled by the governor), so
// it never drags. When a lagging wheel speeds up its PWM falls below saturation and
// the group scale climbs back to 1.0 — self-recovering, no latch.
//
//   cmd[i]    : slewed wheel command, -1000..+1000 (the units pidStep consumes)
//   measTps[] : last measured ticks/sec per wheel (signed, encSign-corrected)
//   outPwm[]  : last commanded PWM per wheel (signed, |.| <= pwmMax)
//   refTps    : UNIFORM target reference — cmd magnitude 1000 == refTps for every
//               wheel (= the weakest wheel's max, so all can reach it). The judge
//               is ABSOLUTE speed against this shared reference: that is what keeps
//               the cart straight when two battery halves differ. (NOT per-wheel
//               max — normalizing each wheel to its own max would call a slow weak
//               wheel "keeping up" and never correct the curve.)
//   pwmMax    : PWM_MAX
//   loScale   : lowest allowed scale, 0..1. 0 => a fully held wheel halts the
//               cart; ~0.1 leaves a little crawl. 1.0 disables it.
//   satFrac   : |out| >= satFrac*pwmMax marks a wheel as maxed-out (giving all it
//               can). ONLY saturated wheels can drag the group (see latch note),
//               so this is also the engage gate. 0..1.
//   valid[]   : false for wheels with no usable encoder (open-loop); skipped.
//               Pass nullptr to treat all four as valid.
//
// Pure integer/float math, no Arduino deps — host-testable.
static inline float speedGovernorScale(const int32_t cmd[4], const float measTps[4],
                                        const float outPwm[4], float refTps, float pwmMax,
                                        float loScale, float satFrac,
                                        const bool valid[4]) {
  float worst = 1.0f;
  for (int i = 0; i < 4; i++) {
    if (valid && !valid[i]) continue;                 // no usable feedback
    float tgt  = ((float)cmd[i] / 1000.0f) * refTps;
    float atgt = tgt < 0 ? -tgt : tgt;
    if (atgt < 0.05f * refTps) continue;              // ~zero demand: ignore

    // Only a saturated wheel can limit the group. An unsaturated wheel has PWM
    // headroom to catch up, or is only slow because THIS scale already throttled
    // it — judging it by that throttled speed is what latched the governor.
    float aout = outPwm[i] < 0 ? -outPwm[i] : outPwm[i];
    if (aout < satFrac * pwmMax) continue;            // headroom / throttled: skip

    float ameas = measTps[i] < 0 ? -measTps[i] : measTps[i];
    float ratio = ameas / atgt;                       // MAGNITUDE: sign-independent
    if (ratio > 1.0f) ratio = 1.0f;
    // A wheel physically turning OPPOSITE its command is not tracking at all.
    bool wrongWay = (tgt > 0.0f && measTps[i] < 0.0f) ||
                    (tgt < 0.0f && measTps[i] > 0.0f);
    if (wrongWay) ratio = 0.0f;

    if (ratio < worst) worst = ratio;
  }
  if (worst < loScale) worst = loScale;
  return worst;
}

// Fade the governor's throttle DEPTH by how rotational the commanded twist is.
//
// WHY: speedGovernorScale judges each wheel against the UNIFORM no-load refTps
// (= maxTpsMin, calibrated wheels-off-ground). Spin-in-place forces the mecanum
// rollers to scrub SIDEWAYS across the floor — the highest-load move — so every
// wheel saturates yet falls well below that no-load target. The governor reads
// this as ALL FOUR wheels failing and throttles the spin toward GOV_FLOOR (crawl /
// surge-and-stall). But symmetric scrub load is exactly NOT the held-corner case
// the governor exists to catch: when every wheel is slow together there is nothing
// to cross-correct. Forward/strafe never trip it (free-rolling wheels reach the
// no-load speed), only rotation does. (Independent of supply voltage — adding a
// battery cannot make a sideways-scrubbing roller reach its free-air rate.)
//
// FIX: scale how much of the throttle survives by the TRANSLATION fraction of the
// commanded twist. Pure spin (omega dominates) -> throttle relaxed back to 1.0
// (governor effectively off); pure translation -> unchanged (full authority); a
// blended diagonal gets a proportional, continuous amount. relax in [0..1] caps
// the maximum relaxation (1 = fully off at pure spin, 0 = feature disabled).
//
//   gScale : the scale from speedGovernorScale (floor..1).
//   vx,vy,omega : the COMMANDED body twist (cmd units).
// Pure float/int math, no Arduino deps — host-testable.
static inline float governorRotationRelax(float gScale, int16_t vx, int16_t vy,
                                          int16_t omega, float relax) {
  float at = (float)((vx < 0 ? -vx : vx) + (vy < 0 ? -vy : vy));
  float aw = (float)(omega < 0 ? -omega : omega);
  float denom = aw + at;
  if (denom < 1.0f) return gScale;            // no meaningful command: leave as-is
  float spin = aw / denom;                    // 0..1, 1 == pure rotation
  float keep = 1.0f - relax * spin;           // fraction of throttle that survives
  return 1.0f - (1.0f - gScale) * keep;       // fade throttle depth toward 1.0
}
