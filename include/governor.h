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
//               wheel (= the weakest wheel's max, so all can reach it). Used only to
//               turn each wheel's command into a target speed; the GROUP scale is
//               judged RELATIVE to the best-tracking wheel, not against this absolute
//               (see GROUP-RELATIVE note below).
//
// GROUP-RELATIVE JUDGE (BUG-001, audit 2026-06-07): a yaw/curve comes from wheels
// turning at DIFFERENT fractions of their target, NOT from all wheels being slow
// together. The earlier judge compared each wheel's |meas|/|tgt| to the no-load
// refTps in ABSOLUTE terms, so any uniform load that held all four wheels below
// their free-air speed (a payload — the highest-load move short of a block) read as
// "all four failing" and throttled the WHOLE twist toward GOV_FLOOR. That crawled
// loaded forward and, worse, made a forward+turn escape the floor only by ADDING
// yaw (non-additive — the reported spin). Fix: scale the worst SATURATED wheel's
// ratio by the BEST tracking wheel's ratio (what is actually achievable right now).
// Uniform load -> worst==best -> scale 1.0 (nothing to cross-correct, the cart is
// already straight at the achievable common speed). A genuinely lagging/held corner
// still tracks far below the others -> worst/best is small -> the group slows to
// match it and stays straight. Pure spin scrubs all four equally -> worst==best ->
// no throttle (so the old spin-crawl is fixed at the root; the rotation relax below
// becomes a secondary backstop, not the primary cure).
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
  float worstSat = 1.0f;   // worst ratio among SATURATED wheels (limits the group)
  float bestAll  = 0.0f;   // best ratio among ALL commanded wheels (the achievable)
  bool  anySat   = false;
  for (int i = 0; i < 4; i++) {
    if (valid && !valid[i]) continue;                 // no usable feedback
    float tgt  = ((float)cmd[i] / 1000.0f) * refTps;
    float atgt = tgt < 0 ? -tgt : tgt;
    if (atgt < 0.05f * refTps) continue;              // ~zero demand: ignore

    float ameas = measTps[i] < 0 ? -measTps[i] : measTps[i];
    float ratio = ameas / atgt;                       // MAGNITUDE: sign-independent
    if (ratio > 1.0f) ratio = 1.0f;
    // A wheel physically turning OPPOSITE its command is not tracking at all.
    bool wrongWay = (tgt > 0.0f && measTps[i] < 0.0f) ||
                    (tgt < 0.0f && measTps[i] > 0.0f);
    if (wrongWay) ratio = 0.0f;

    // Every commanded wheel contributes to "what is achievable right now" — a
    // wheel with PWM headroom that is tracking well sets the reference the laggards
    // are judged against, so uniform load (all equally slow) does not throttle.
    if (ratio > bestAll) bestAll = ratio;

    // Only a SATURATED wheel can DRAG the group: an unsaturated wheel has headroom
    // to catch up, or is only slow because THIS scale already throttled it (judging
    // that throttled speed is what latched the governor at the floor).
    float aout = outPwm[i] < 0 ? -outPwm[i] : outPwm[i];
    if (aout < satFrac * pwmMax) continue;            // headroom / throttled: skip
    anySat = true;
    if (ratio < worstSat) worstSat = ratio;
  }

  if (!anySat) return 1.0f;                           // nothing maxed-out -> no drag
  if (bestAll < 0.05f) return loScale;                // everything failing -> floor
  // Group-relative: slow to the laggard's share of the best-tracking wheel. Uniform
  // load -> worstSat==bestAll -> 1.0; a lagging corner -> worstSat/bestAll < 1.
  float scale = worstSat / bestAll;
  if (scale > 1.0f) scale = 1.0f;
  if (scale < loScale) scale = loScale;
  return scale;
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
//   relax  : max relaxation depth (1 = governor fully off at pure spin; 0 = off).
//   transW : translation de-weight in the spin metric, 0..1 (BUG-002). The scrub-
//            induced false-throttle is created by the ROTATION component, but the
//            old metric spin = aw/(aw+at) divided relief by TOTAL command, so a
//            translation-dominant blend (forward + small turn) got almost none —
//            adding a small turn under load left a deep throttle on the forward
//            part (non-additive speed). De-weighting translation in the denominator
//            (spin = aw/(aw + transW*at)) gives such blends proportionally more
//            relief while leaving the endpoints identical: pure spin (at=0) -> 1,
//            pure translate (aw=0) -> 0. transW=1 reproduces the old fraction.
// Pure float/int math, no Arduino deps — host-testable.
static inline float governorRotationRelax(float gScale, int16_t vx, int16_t vy,
                                          int16_t omega, float relax, float transW) {
  float at = (float)((vx < 0 ? -vx : vx) + (vy < 0 ? -vy : vy));
  float aw = (float)(omega < 0 ? -omega : omega);
  float denom = aw + transW * at;
  if (denom < 1.0f) return gScale;            // no meaningful command: leave as-is
  float spin = aw / denom;                    // 0..1, 1 == pure rotation
  if (spin > 1.0f) spin = 1.0f;
  float keep = 1.0f - relax * spin;           // fraction of throttle that survives
  return 1.0f - (1.0f - gScale) * keep;       // fade throttle depth toward 1.0
}
