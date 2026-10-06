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
//   cmd[i]    : slewed wheel command BEFORE this scale, -1000..+1000 (pidStep units)
//   measTps[] : last measured ticks/sec per wheel (signed, encSign-corrected)
//   outPwm[]  : last commanded PWM per wheel (signed, |.| <= pwmMax)
//   refTps    : UNIFORM target reference — cmd magnitude 1000 == refTps for every
//               wheel (= SPEED_REF_FRAC x the weakest wheel's max, so all can reach
//               it with headroom; config_robot.h cmdRefTps()). Used only to
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
// REVERSAL (audit 2026-10-05): when EVERY commanded wheel is turning against its
// command, the cart is braking through a deliberate reversal (stick yanked from
// forward to reverse), not failing. Judging that as "all wheels at ratio 0" floored
// the group to loScale and cut the braking command to 10% exactly during the
// emergency move (stops ~7-10% longer in simulation). That case now returns 1.0.
// A SINGLE wrong-way wheel among tracking ones is still a dragged/held corner and
// still floors the group.
//
// GOVERNED-TARGET JUDGE (strafe yaw fix 2026-10-05). Ratios used to be judged
// against the UN-throttled command, so a wheel tracking its THROTTLED target read
// ratio = current scale and the best wheel's ratio fell with the scale. With one
// pair pinned at its ceiling the scale settled at sqrt(laggard ratio), not the
// ratio: the strong pair kept running ahead of the weak one. In a strafe that gap
// between the front and rear pairs IS yaw (mecanumMix's omega column); in forward
// drive the same gap only lands on the null axis, which is why strafing was the
// move that swung the nose round. The rider's rear pair runs on the weaker pack
// (config_robot.h MAX_TPS) and carries the rider, so it is the pair that pins.
// Every ratio is now judged against the target the wheel was actually asked for
// (cmd x curScale), and a lagging pinned wheel moves the group to its real pace in
// one step: curScale x (worst/best) / (1 - tol), which parks it just inside the
// tolerance instead of exactly at its ceiling. Two rules keep that from ratcheting
// speed down under heavy load (what sank the earlier attempt at this judge):
//   - a pinned wheel within `tol` of the best wheel is keeping up and does not
//     drag (encoder noise alone cannot walk the scale down);
//   - a pinned wheel that keeps up lets the scale creep back up by `probeStep`
//     per call, so the group re-finds the laggard's ceiling when load eases. With
//     no pinned wheel the scale releases to 1.0 as before.
// Steady state: the laggard runs at its ceiling and the others within `tol` of it.
//
// Pure integer/float math, no Arduino deps — host-testable.
//   curScale  : the scale applied to the commands this measurement came from.
//   tol       : lag (fraction of the best wheel's tracking) a pinned wheel may
//               show before it drags the group. Keep above encoder noise.
//   probeStep : scale increase per call while a pinned wheel keeps up.
static inline float speedGovernorScale(const int32_t cmd[4], const float measTps[4],
                                        const float outPwm[4], float refTps, float pwmMax,
                                        float loScale, float satFrac,
                                        const bool valid[4],
                                        float curScale, float tol, float probeStep) {
  float worstSat = 1.0f;   // worst ratio among SATURATED wheels (limits the group)
  float bestAll  = 0.0f;   // best ratio among ALL commanded wheels (the achievable)
  bool  anySat   = false;
  int   nCmd = 0, nWrong = 0;   // commanded wheels / of those, turning against it
  const float asked = curScale > 0.01f ? curScale : 0.01f;   // GOV_FLOOR 0 is legal
  for (int i = 0; i < 4; i++) {
    if (valid && !valid[i]) continue;                 // no usable feedback
    float tgt  = ((float)cmd[i] / 1000.0f) * refTps;
    float atgt = tgt < 0 ? -tgt : tgt;
    if (atgt < 0.05f * refTps) continue;              // ~zero demand: ignore

    float ameas = measTps[i] < 0 ? -measTps[i] : measTps[i];
    float ratio = ameas / (atgt * asked);             // MAGNITUDE, vs the governed target
    if (ratio > 1.0f) ratio = 1.0f;
    // A wheel physically turning OPPOSITE its command is not tracking at all.
    bool wrongWay = (tgt > 0.0f && measTps[i] < 0.0f) ||
                    (tgt < 0.0f && measTps[i] > 0.0f);
    nCmd++;
    if (wrongWay) { ratio = 0.0f; nWrong++; }

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

  if (nCmd > 0 && nWrong == nCmd) return 1.0f;        // braking through a reversal
  if (!anySat) return 1.0f;                           // nothing maxed-out -> no drag
  if (bestAll < 0.05f) return loScale;                // everything failing -> floor
  // Group-relative: slow to the laggard's share of the best-tracking wheel. Uniform
  // load -> worstSat==bestAll -> no drag; a lagging corner -> worstSat/bestAll < 1.
  float lag = worstSat / bestAll;
  float keep = 1.0f - tol;                             // tolerated lag
  float scale = lag < keep ? curScale * lag / keep     // slow the group to its pace
                           : curScale + probeStep;     // keeping up: creep back up
  if (scale > 1.0f) scale = 1.0f;
  if (scale < loScale) scale = loScale;
  return scale;
}

// Fade the governor's throttle DEPTH by how rotational the commanded twist is.
//
// WHY: spin-in-place forces the mecanum rollers to scrub SIDEWAYS across the
// floor — the highest-load move. When the governor judged wheels against the
// absolute no-load reference, every wheel read as failing and the spin was
// throttled toward GOV_FLOOR. Since BUG-001 the judge is group-relative
// (symmetric scrub -> worst == best -> no throttle), so this relax is now a
// secondary backstop for ASYMMETRIC scrub during rotation, not the primary cure.
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
