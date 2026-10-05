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
