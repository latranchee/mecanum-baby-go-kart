#pragma once
#include <stdint.h>

// ESP-NOW packet: controller -> robot
// Send rate ~50Hz. Robot watchdog stops motors if no packet for 500ms.
struct __attribute__((packed)) CtrlPacket {
  uint32_t seq;       // monotonic counter (debug + dedup)
  int16_t  vx;        // -1000..+1000 (forward+)
  int16_t  vy;        // -1000..+1000 (strafe right+)
  int16_t  omega;     // -1000..+1000 (CCW+)
  uint8_t  buttons;   // bit0=LeftBtn, bit1=RightBtn, bit2=LeftJoyBtn, bit3=RightJoyBtn
  uint8_t  flags;     // bit0=estop
};

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
