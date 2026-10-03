#pragma once
#include <stdint.h>

// ESP-NOW packet: controller -> robot
// Send rate ~50Hz. Robot watchdog stops motors if no packet for 500ms.
// Robot drops frames whose seq is not newer (dedup) or whose crc8 mismatches.
struct __attribute__((packed)) CtrlPacket {
  uint32_t seq;       // monotonic counter; robot accepts only newer seq (dedup)
  int16_t  vx;        // -1000..+1000 (forward+)
  int16_t  vy;        // -1000..+1000 (strafe right+)
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

// Robot MAC + ESP-NOW keys live in secrets.h (gitignored). A fresh clone with no
// secrets.h still builds via the fallback below. Copy secrets.h.example ->
// secrets.h and fill in your robot's real MAC (and keys, for #3 encryption).
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  static const uint8_t ROBOT_MAC[6] = { 0x5C, 0x01, 0x3B, 0x34, 0xDB, 0x18 };
#endif

// Shared WiFi channel for ESP-NOW (no AP needed, but channel must match)
static const uint8_t ESPNOW_CHANNEL = 1;

// ESP-NOW link encryption (#3). 0 = plaintext (default — no key coordination
// needed). To enable: set to 1 on BOTH ends, flash robot+controller back-to-back
// from the SAME secrets.h (ESPNOW_PMK/LMK + CONTROLLER_MAC), and verify ONLINE
// before walking away. Mismatched/missing keys silently kill the link. Roll back
// by setting this to 0 and reflashing.
#ifndef ESPNOW_ENCRYPT
#define ESPNOW_ENCRYPT 0
#endif
