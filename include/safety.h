#pragma once
#include <stdint.h>
#include <string.h>

// Robot-side link + drive safety helpers. Pure logic, no Arduino deps —
// host-testable (test/test_safety).

// Age of a millis() stamp. The ESP-NOW callback runs on core 0 and can stamp a
// packet AFTER the drive loop on core 1 read `now`; a plain unsigned `now - last`
// then wraps to ~4e9 ms, reads as a dead link and hard-stops the cart for a tick.
// A stamp from the future is simply fresh.
static inline uint32_t linkAgeMs(uint32_t now, uint32_t last) {
  int32_t d = (int32_t)(now - last);
  return d < 0 ? 0u : (uint32_t)d;
}

// Receive gate for control frames: one locked sender, newer seq only.
//
// While the link is live the robot follows ONE transmitter (the MAC of the frame
// that opened the session) and accepts only a newer seq from it (wrap-safe). A
// second transmitter (headset left on, another ESP-NOW project on channel 1) is
// dropped instead of interleaving with the first.
//
// After `resyncMs` of silence the gate reopens: the next valid frame is accepted
// whatever its seq or sender, and locks the session to that sender. A rebooted
// controller restarts its seq near 0, and its boot always takes longer than the
// watchdog, so silence is what proves the restart — the motors are already stopped
// by then. (The old rule, "a backward jump >= 1000 means reboot", locked out a
// controller power-cycled within ~20 s of its own power-on while its screen still
// showed ONLINE.)
struct LinkGate {
  uint8_t  mac[6];
  uint32_t lastSeq;
  uint32_t lastMs;   // millis() of the last accepted frame
  bool     locked;   // false until the first frame (zero-init = open)
};

enum LinkVerdict : uint8_t { LINK_ACCEPT = 0, LINK_STALE = 1, LINK_FOREIGN = 2 };

static inline LinkVerdict linkGateCheck(LinkGate* g, const uint8_t mac[6], uint32_t seq,
                                        uint32_t nowMs, uint32_t resyncMs) {
  bool reopen = !g->locked || linkAgeMs(nowMs, g->lastMs) >= resyncMs;
  if (!reopen) {
    if (memcmp(mac, g->mac, 6) != 0) return LINK_FOREIGN;
    if ((int32_t)(seq - g->lastSeq) <= 0) return LINK_STALE;
  } else {
    memcpy(g->mac, mac, 6);
    g->locked = true;
  }
  g->lastSeq = seq;
  g->lastMs  = nowMs;
  return LINK_ACCEPT;
}

// Drive fault: a wheel driven hard with no encoder motion.
//
// Catches a dead encoder (loose encoder supply — it happened on FR 2026-05-31):
// the wheel reads 0 ticks, its PI loop winds it to near-full PWM, the governor
// crawls the other three, and the cart spins hard about the blind corner. Also
// catches a jammed wheel, which otherwise holds up to ~73% PWM into a stalled motor
// indefinitely. Both look identical from the firmware and both want the power cut.
//
//   pwm, measTps : this tick's delivered PWM and measured speed (signed)
//   pwmGate      : |pwm| at or above this counts as "driven hard"
//   tpsGate      : |measTps| below this counts as "not moving"
//   tripMs       : how long the pair must hold
// Returns true once tripped. f->ms resets the moment either condition clears.
struct DriveFault { float ms; };

static inline bool driveFaultStep(DriveFault* f, float pwm, float measTps,
                                  float pwmGate, float tpsGate, float tripMs, float dtMs) {
  float ap = pwm < 0 ? -pwm : pwm;
  float am = measTps < 0 ? -measTps : measTps;
  if (ap >= pwmGate && am < tpsGate) f->ms += dtMs;
  else                               f->ms = 0.0f;
  return f->ms >= tripMs;
}

// Wheel recovery: when a switched-off (faulted) wheel gets power back WITHOUT the
// operator stopping. Olivier 2026-10-05: "once it's declared dead it doesn't come
// back ever" — the neutral-sticks re-arm alone never fires while driving.
//   - FREE ROLL: the unpowered wheel's encoder shows it turning (>= rollGate for
//     rollMs): on the floor a freed wheel rolls with the cart, which proves both
//     that it is no longer jammed and that its encoder works. Re-arm at once.
//   - RETRY: otherwise re-arm after delayMs. Each trip shortly after a re-arm
//     doubles the delay (first..max); a wheel that then runs healthy for
//     healthyResetMs starts over at `first`. A truly dead encoder therefore costs a
//     short re-trip (<= ~0.4 s of that wheel) at most every `max` ms.
// The neutral-sticks re-arm (DriveGate) still re-arms everything at once.
// Pure logic, host-tested.
struct WheelRecovery {
  bool     off;        // switched off by a drive fault
  bool     everTripped;
  uint32_t offAtMs;    // when it was (last) switched off
  uint32_t armedAtMs;  // when it was last re-armed
  uint32_t delayMs;    // current retry delay
  float    rollMs;     // how long the unpowered wheel has been seen rolling
};

struct WheelRecoveryCfg {
  uint32_t firstMs, maxMs, healthyResetMs;
  float    rollGateTps, rollMs;
};

static inline void wheelRecoveryTrip(WheelRecovery* r, uint32_t nowMs, const WheelRecoveryCfg& c) {
  const bool recent = r->everTripped && (uint32_t)(nowMs - r->armedAtMs) < c.healthyResetMs;
  if (!recent)                 r->delayMs = c.firstMs;
  else if (r->delayMs < c.maxMs) {
    uint32_t d = r->delayMs * 2;
    r->delayMs = d > c.maxMs ? c.maxMs : d;
  }
  r->off = true;
  r->everTripped = true;
  r->offAtMs = nowMs;
  r->rollMs = 0.0f;
}

static inline void wheelRecoveryRearm(WheelRecovery* r, uint32_t nowMs) {
  r->off = false;
  r->armedAtMs = nowMs;
  r->rollMs = 0.0f;
}

// Call every tick for an OFF wheel with its (unpowered) measured speed.
// 0 = stay off, 1 = re-arm: rolling freely, 2 = re-arm: retry delay elapsed.
static inline int wheelRecoveryStep(WheelRecovery* r, uint32_t nowMs, float measTps,
                                    float dtMs, const WheelRecoveryCfg& c) {
  if (!r->off) return 0;
  const float am = measTps < 0 ? -measTps : measTps;
  r->rollMs = am >= c.rollGateTps ? r->rollMs + dtMs : 0.0f;
  if (r->rollMs >= c.rollMs) return 1;
  if ((uint32_t)(nowMs - r->offAtMs) >= r->delayMs) return 2;
  return 0;
}

// Drive gate: the single "may the motors run this tick?" decision, plus the two
// latches that need neutral sticks to release.
//
//   fault     : set by driveGateTrip() when a drive fault switches a wheel off.
//               It does NOT stop the drive: the faulted wheel is switched off and
//               the other wheels keep driving (Olivier, 2026-10-05: "show an error
//               for that wheel, not stop the whole thing"). The latch only decides
//               when the switched-off wheel(s) get re-armed: after clearMs of
//               neutral sticks (or e-stop) on a LIVE link.
//   linkLatch : set whenever the radio link is lost (watchdog). Clears on the
//               first fresh frame that is neutral (or e-stop). Without it, a
//               marginal link stopped and relaunched the cart at whatever the
//               stick held, over and over. Also set at boot (no frame yet), so
//               the first command a robot obeys is a neutral one.
//
// Serial test mode has its own link watchdog (test-link) and a forced-fresh
// packet, so the link latch neither sets nor holds there. Pure logic, host-tested.
struct DriveGate {
  bool     fault;
  bool     linkLatch;
  uint32_t idleSinceMs;   // start of the neutral window that clears a fault
};

struct DriveGateIn {
  bool encReady;   // all encoder units counting
  bool linkOk;     // fresh frame within the watchdog
  bool estop;      // e-stop flag on the current frame
  bool neutral;    // current command is zero (twist and any direct PWM)
  bool testMode;   // serial bench mode owns the drive
};

struct DriveGateOut {
  bool stop;          // hold ALL motors stopped this tick (a wheel fault alone never does)
  bool faultCleared;  // the fault latch released on this tick: re-arm the wheel(s)
  bool linkLatched;   // the link latch engaged on this tick
  bool linkReleased;  // the link latch released on this tick
};

static inline void driveGateTrip(DriveGate* g, uint32_t nowMs) {
  g->fault       = true;
  g->idleSinceMs = nowMs;
}

static inline DriveGateOut driveGateStep(DriveGate* g, const DriveGateIn& in,
                                         uint32_t nowMs, uint32_t clearMs) {
  DriveGateOut o = { false, false, false, false };
  const bool idle = in.linkOk && (in.estop || in.neutral);

  if (in.testMode) {
    if (g->linkLatch) { g->linkLatch = false; o.linkReleased = true; }
  } else if (!in.linkOk) {
    if (!g->linkLatch) { g->linkLatch = true; o.linkLatched = true; }
  } else if (g->linkLatch && idle) {
    g->linkLatch = false;
    o.linkReleased = true;
  }

  if (g->fault) {
    if (!idle) g->idleSinceMs = nowMs;
    else if ((uint32_t)(nowMs - g->idleSinceMs) >= clearMs) {
      g->fault = false;
      o.faultCleared = true;
    }
  }

  o.stop = !in.encReady || !in.linkOk || in.estop ||
           (g->linkLatch && !in.testMode);
  return o;
}
