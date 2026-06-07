#pragma once
#include <stdint.h>

// Robot tunables (firmware-only). Pure constants — logic lives in kinematics.h /
// control_math.h. Keep hardware/pin facts in src/robot/main.cpp.

// PWM
static const int PWM_FREQ = 25000;
static const int PWM_RES  = 10;
static const int PWM_MAX  = (1 << PWM_RES) - 1;   // 1023
static const int DEADBAND = 0;  // disabled: PID feed-forward handles low-speed PWM

// Velocity control
// Per-wheel full-PWM tick rate (ticks/sec) — slot order [FL,FR,RL,RR].
// 4x quadrature decode (BUG-008 fixed): every A/B edge counts, so the tick rate
// is ~4x the old single-edge value. These MUST be re-measured on hardware after
// the decode change AND whenever the supply changes — with two battery halves of
// different voltage the halves WILL differ, and setting each wheel to ITS own max
// is what lets feed-forward + the governor normalize correctly (the whole point
// of this change). Calibrate with tools/calibrate_maxtps.py (wheels off ground).
// Feed-forward gain is derived per wheel as PWM_MAX / MAX_TPS[i] at use site.
// Measured 2026-06-02 (4x decode, no-load, full PWM, min of fwd/rev per wheel):
// wheels within ~5% — packs well matched at no-load. RR weakest = uniform ref.
static const float MAX_TPS[4] = { 8275.0f, 8483.0f, 8342.0f, 8230.0f };
// Smallest per-wheel max — used where one scalar reference is still needed.
static inline float maxTpsMin() {
  float m = MAX_TPS[0];
  for (int i = 1; i < 4; i++) if (MAX_TPS[i] < m) m = MAX_TPS[i];
  return m;
}
static const float Kp      = 0.15f;
static const float Ki      = 0.3f;                      // glitch rejection now guards windup, so raise Ki back up to regulate weak/loaded wheels (was under-driving right side -> drift)
static const float I_MAX   = 0.55f * (float)PWM_MAX;    // bound integral authority (~563)

// Command slew limiter (cmd units/sec). Gentle + long ramp: target rises slowly
// enough that measured tracks it, so the integral never over-winds during accel
// (that windup was making light-loaded wheels overshoot then settle back).
static const float CMD_SLEW = 1500.0f;  // full 0..1000 in ~670ms

// Output PWM slew limiter (PWM units/sec). Caps how fast per-wheel PWM can
// change. This is the direct lever against break-away overshoot: a cold motor
// can't jump to full torque and overspeed before the loop reins it in — PWM
// physically can't rise faster than this. Also kills the rotate limit cycle.
static const float PWM_SLEW = 2500.0f;  // full 0..1023 in ~410ms

// Cross-wheel speed governor (electronic differential) — governor.h.
// Independent per-wheel PI loops never coordinate, so holding/stalling one wheel
// lets the others keep full speed and the cart yaws about the held corner. When
// enabled, each tick scales ALL wheel commands by the worst-tracking wheel so the
// group slows to match the one that can't keep up (held, stalled, traction- or
// supply-limited) and the cart stays pointed straight.
//   GOV_FLOOR  : lowest group scale. 0.0 => a held wheel halts the cart; 0.10
//                leaves a crawl. Set 1.0 (or SYNC_GOVERNOR 0) to disable.
//   GOV_SAT_FRAC: |out| >= this fraction of PWM_MAX marks a wheel saturated. ONLY
//                saturated wheels can drag the group (an unsaturated wheel has
//                headroom or is merely throttled) — this is the engage gate and
//                the fix for the old latch where forward drive stuck at the floor.
//   GOV_SLEW   : how fast the applied scale may move (1/sec), low-passed so a
//                transient startup lag can't collapse drive and recovery is smooth.
#ifndef SYNC_GOVERNOR
#define SYNC_GOVERNOR 1
#endif
static const float GOV_FLOOR    = 0.10f;
static const float GOV_SAT_FRAC = 0.85f;  // |out| >= 85% PWM_MAX = wheel maxed out
// Asymmetric slew: drop FAST so a sudden block/stall is caught in a few ticks,
// recover SLOWLY so drive eases back smoothly without a lurch. (BUG-008) The old
// 12-vs-3 ratio (4x) let govScale RATCHET toward the floor under a train of brief
// turn-induced saturations: it fell four times faster than it could climb back, so
// the effective forward scale sat far below the average achievable speed. Halving
// the asymmetry (12 vs 6) keeps fast block-catching while letting drive recover
// between micro-saturations so a loaded forward+turn does not pin to a crawl.
static const float GOV_SLEW_DOWN = 12.0f;  // full 1->0 in ~85 ms (catch blocks)
static const float GOV_SLEW_UP   = 6.0f;   // full 0->1 in ~165 ms (recover, no ratchet)
// Rotation relax (governor.h governorRotationRelax): spin-in-place scrubs all four
// rollers sideways below the no-load refTps, so the governor reads universal
// saturation and throttles the spin to a crawl — yet symmetric load is NOT the
// held-corner case it exists for. Fade the throttle DEPTH by the rotation fraction
// of the commanded twist: pure spin => governor off, pure translate => full
// authority, diagonal => proportional. 1.0 = fully relax at pure spin; 0 disables.
// (BUG-002) Lowered 1.0 -> 0.5: the relax exists to stop the governor crawling a
// PURE spin (symmetric scrub reads as universal failure). But with the cross-wheel
// judge now RELATIVE to the best-tracking wheel (governor.h), symmetric load no
// longer floors the group at all, so the relax is a secondary backstop, not the
// primary spin fix. At 1.0 it could FULLY disarm the governor on a forward+turn
// blend — exactly when a loaded outer corner lags asymmetrically and the governor
// SHOULD slow the group to stay straight. Capping relaxation depth at 0.5 keeps a
// little spin relief while never fully removing straightness protection on a curve.
static const float GOV_SPIN_RELAX = 0.5f;
// Translation de-weight in the spin metric (BUG-002). <1 gives a forward+turn
// blend more of the scrub relief that the rotation component warrants, instead of
// dividing relief by total command magnitude (which left translation-dominant
// blends deeply throttled = non-additive speed). 1.0 = old fraction behaviour.
static const float GOV_SPIN_RELAX_TRANS_W = 0.5f;

// Body-space outer loop (body_loop.h) — motion-correct compensation.
// Closes a slow heading/centre loop in body space (vx,vy,omega) ON TOP of the
// inner per-wheel PI + governor. A per-wheel SISO loop can't tell "one wheel
// lagging" from "the chassis yawing"; the governor can only throttle the whole
// twist uniformly. This estimates the chassis twist from the wheels and adds a
// small corrective twist: yaw-hold dominates when translating, centre-hold when
// rotating (the continuous weight IS the per-motion mode selector). The
// correction bypasses CMD_SLEW and is added AFTER the governor scale, so its yaw
// authority survives while the governor throttles base magnitude.
//   BODY_IIR_ALPHA_* : single-pole IIR on the body estimate (filtered += a*(new-
//                      filtered)). Smaller = heavier filter; omega/vy are noisier
//                      than vx so they get the heavier (smaller-alpha) filter.
//   BODY_KP_*/KI_*   : per-axis P/I. Outer ~3-5x slower than the inner PID. Start
//                      P-only on yaw (KI small) and raise KI only if steady yaw
//                      drift persists. KI terms are clamped to BODY_I_MAX.
//   BODY_W_THRESH    : |omega_cmd| above which motion counts as "rotating".
//   BODY_RATE        : correction slew (cmd units/sec) — anti-jerk only.
//   BODY_SLIP_GATE / BODY_GOV_FREEZE : freeze the outer integral when |s| (slip)
//                      is high or the governor is throttling (handoff invariant).
#ifndef BODY_LOOP
#define BODY_LOOP 1
#endif
static const float BODY_IIR_ALPHA_TRANS = 0.50f;  // vx estimate (lighter filter)
static const float BODY_IIR_ALPHA_W     = 0.30f;  // omega/vy estimate (heavier)
static const float BODY_KP_W      = 0.28f;
static const float BODY_KP_TRANS  = 0.15f;
static const float BODY_KI_W      = 0.15f;        // /s, clamped
static const float BODY_KI_TRANS  = 0.10f;        // /s, clamped
static const float BODY_I_MAX     = 150.0f;       // ~15% of 1000 cmd authority
static const float BODY_CORR_MAX  = 200.0f;       // hard per-axis correction clamp
// (BUG-004/005) Relative yaw-correction cap. The absolute BODY_CORR_MAX alone let a
// ±200 yaw differential equal or exceed a governor-throttled forward common-mode,
// and fading the correction by govScale is a NO-OP on the yaw:forward RATIO (both
// base and correction scale by it, so it cancels). While TRANSLATING (yaw-hold
// regime) the yaw correction is additionally bounded to this fraction of the
// current throttled forward authority, so a curve can never be turned into a spin:
// yaw stays a minority of forward no matter how far the governor has shrunk it.
static const float BODY_YAW_FWD_FRAC = 0.5f;      // yaw corr <= 50% of throttled fwd
// (BUG-007) Outer-integral decay rate (per second) applied WHILE the integral is
// frozen (governor owns magnitude). Without it, iw winds during unfrozen windows
// and HOLDS its wound value across the 0.95 freeze threshold, then dumps as a
// heading kick when the threshold is recrossed. Bleeding it toward zero while
// frozen removes the windup/dump so no standing yaw bias survives a throttle dip.
static const float BODY_I_DECAY   = 3.0f;         // frozen integral bleed, 1/s
// |omega_cmd| (cmd units, axis is +-1000) at which motion counts fully as
// "rotating". Was 150 (BUG-008): on a +-1000 axis that hard-switched the loop
// into centre-hold (and zeroed yaw-hold) by only 15% stick, so a deliberate
// modest turn snapped the weighting regime. 500 makes the translate<->rotate
// blend span half the axis — a genuine continuous mode selector, with yaw-hold
// still meaningfully active through a normal curve.
static const float BODY_W_THRESH  = 500.0f;       // |omega_cmd| => fully "rotating"
static const float BODY_RATE      = 400.0f;       // correction slew, cmd units/sec
static const float BODY_GOV_FREEZE = 0.95f;       // govScale below this freezes integral
static const float BODY_SLIP_GATE  = 250.0f;      // |s| (cmd units) above this freezes integral

// Stall detection (telemetry only): flag a wheel pinned near max PWM while barely
// moving (held/jammed/supply-collapsed) so the operator can see WHICH wheel.
static const float STALL_PWM_FRAC = 0.85f;   // |out| above this fraction of PWM_MAX
static const float STALL_TPS_FRAC = 0.08f;   // |measured| below this fraction of MAX_TPS
static const float STALL_MS       = 300.0f;  // sustained for this long

// Link watchdog: stop motors if no fresh packet for this long.
static const uint32_t WATCHDOG_MS = 500;

// Battery sensing (#5). No divider wired by default -> disabled (pin -1), so no
// fabricated voltage is logged. To enable: set BATT_ADC_PIN to the ADC-capable
// GPIO reading the divider, and BATT_DIVIDER to Vbatt/Vadc (the divider ratio).
static const int   BATT_ADC_PIN = -1;
static const float BATT_DIVIDER = 1.0f;
