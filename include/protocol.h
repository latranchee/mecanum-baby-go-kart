#pragma once
#include <stdint.h>
#include <stddef.h>
#include "control_math.h"   // crc8()

// ESP-NOW packet: controller -> robot
// Send rate ~50Hz. Robot watchdog stops motors if no packet for 500ms.
// Robot drops frames whose crc8 mismatches, frames from a second transmitter while
// the link is live, and frames whose seq is not newer (dedup). After 500 ms of
// silence it accepts the next valid frame from anyone (safety.h linkGateCheck).
struct __attribute__((packed)) CtrlPacket {
  uint32_t seq;       // monotonic counter; robot accepts only newer seq (dedup)
  // Twist in the ROBOT FIRMWARE frame. The rider faces the firmware's rear, so the
  // controller sends rider-forward as vx < 0 and rider-right as vy < 0
  // (config_controller.h INVERT_VX/VY). Rotation is the same in both frames.
  int16_t  vx;        // -1000..+1000 (firmware forward+)
  int16_t  vy;        // -1000..+1000 (firmware strafe right+)
  int16_t  omega;     // -1000..+1000 (CCW+)
  uint8_t  buttons;   // bit0=LeftBtn, bit1=RightBtn, bit2=LeftJoyBtn, bit3=RightJoyBtn
  uint8_t  flags;     // bit0=estop; bits1-3 = feature DISABLE bits (see below)
  uint8_t  crc;       // crc8 over all preceding bytes (control_math.h). MUST be last.
};

// flags bits. bit0 = estop (existing). The mode bits are DISABLE bits so that
// flags==0 means "all features ON" — i.e. the current/legacy behaviour. Any
// sender that leaves flags clear (headset, an un-updated controller) keeps every
// feature enabled, so the wire stays backward-compatible (layout/size unchanged).
#define CTRL_FLAG_ESTOP      0x01
#define CTRL_FLAG_GOV_OFF    0x02   // cross-wheel governor disabled
#define CTRL_FLAG_CL_OFF     0x04   // per-wheel closed-loop disabled (open-loop FF)
#define CTRL_FLAG_BODY_OFF   0x08   // body-space outer loop disabled

// Host-testable preset -> flags builder. `disableBits` is an OR of the *_OFF bits
// above; `estop` ORs in the estop bit. Kept tiny + header-only so both the
// controller firmware and the native unit test share one definition.
static inline uint8_t ctrlFlagsFromPreset(uint8_t disableBits, bool estop) {
  uint8_t f = disableBits & (CTRL_FLAG_GOV_OFF | CTRL_FLAG_CL_OFF | CTRL_FLAG_BODY_OFF);
  if (estop) f |= CTRL_FLAG_ESTOP;
  return f;
}

// Wire-format guard. The 13-byte layout is shared verbatim by every TX (robot,
// controller, headset) and the robot's onRecv rejects any frame whose length !=
// sizeof(CtrlPacket) — so a silent struct-size drift (added field, alignment
// change) kills the whole link with zero diagnostics. Fail the build instead.
static_assert(sizeof(CtrlPacket) == 13, "CtrlPacket wire format must stay 13 bytes");

// ESP-NOW status packet: robot -> controller (back-channel, ~5 Hz), sent to
// whichever transmitter the robot's receive gate is locked onto. Lets the
// controller show what the robot is doing instead of a bare ONLINE: which wheel a
// drive fault switched off, and when the robot is waiting for neutral sticks.
// Different length from CtrlPacket, so neither side can mistake one for the other.
struct __attribute__((packed)) StatusPacket {
  uint8_t magic;      // STATUS_MAGIC
  uint8_t seq;        // wraps; informational only
  uint8_t faultMask;  // bit i = motor slot i switched off by a drive fault
  uint8_t flags;      // STATUS_FLAG_* below
  uint8_t crc;        // crc8 over all preceding bytes. MUST be last.
};
static_assert(sizeof(StatusPacket) == 5, "StatusPacket wire format must stay 5 bytes");

#define STATUS_MAGIC             0x5A
#define STATUS_FLAG_LINK_LATCH   0x01   // waiting for a neutral frame after a link loss
#define STATUS_FLAG_TEST_MODE    0x02   // serial bench mode owns the drive
#define STATUS_FLAG_ENC_DOWN     0x04   // encoder init failed: drive disabled

static inline StatusPacket statusPacketMake(uint8_t seq, uint8_t faultMask, uint8_t flags) {
  StatusPacket p = { STATUS_MAGIC, seq, (uint8_t)(faultMask & 0x0F), flags, 0 };
  p.crc = crc8((const uint8_t*)&p, offsetof(StatusPacket, crc));
  return p;
}

// True (and *out filled) only for a well-formed status frame.
static inline bool statusPacketParse(const uint8_t* data, int len, StatusPacket* out) {
  if (len != (int)sizeof(StatusPacket)) return false;
  StatusPacket p;
  for (size_t i = 0; i < sizeof(p); i++) ((uint8_t*)&p)[i] = data[i];
  if (p.magic != STATUS_MAGIC) return false;
  if (p.crc != crc8((const uint8_t*)&p, offsetof(StatusPacket, crc))) return false;
  *out = p;
  return true;
}

// The rider's name for each motor slot. Slots are named in the firmware frame
// (FL FR RL RR), but the rider faces the firmware's rear (config_controller.h
// INVERT_VX/VY); bench-confirmed 2026-10-05: slot 0 is the rider's rear-right.
// Everything shown to a human uses these.
static const char* const SLOT_RIDER_NAME[4] = { "RR", "RL", "FR", "FL" };

// Robot MAC + ESP-NOW keys live in secrets.h (gitignored). A fresh clone with no
// secrets.h still builds via the fallback below. Copy secrets.h.example ->
// secrets.h and fill in your robot's real MAC (and keys, for #3 encryption).
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  static const uint8_t ROBOT_MAC[6] = { 0x5C, 0x01, 0x3B, 0x34, 0xDB, 0x18 };
#endif

// Shared WiFi channel for ESP-NOW (no AP needed, but channel must match).
// Channel 1 is also a default for many home routers. If the robot log shows
// " (stale)" lines or the controller flickers OFFLINE near the house Wi-Fi, check
// the router's channel and move this to the one farthest from it (Canada: 1/6/11),
// then flash robot and controller together.
static const uint8_t ESPNOW_CHANNEL = 1;

// ESP-NOW link encryption (#3). 0 = plaintext (default — no key coordination
// needed). To enable: set to 1 on BOTH ends, flash robot+controller back-to-back
// from the SAME secrets.h (ESPNOW_PMK/LMK + CONTROLLER_MAC), and verify ONLINE
// before walking away. Mismatched/missing keys silently kill the link. Roll back
// by setting this to 0 and reflashing.
#ifndef ESPNOW_ENCRYPT
#define ESPNOW_ENCRYPT 0
#endif
