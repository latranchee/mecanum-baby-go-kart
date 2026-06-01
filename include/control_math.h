#pragma once
#include <stdint.h>
#include <stddef.h>

// Pure host-safe helpers shared by firmware and native tests. No Arduino deps.

static inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Map a raw 12-bit stick reading to [-1000..+1000] about its calibrated center,
// with a dead band and linear scaling over the usable half-range. Replaces the
// Arduino constrain() macro with clampI32 so it builds native.
static inline int16_t normalize(uint16_t raw, uint16_t center,
                                int16_t deadzoneRaw, int32_t halfRange) {
  int32_t d = (int32_t)raw - (int32_t)center;
  if (d > -deadzoneRaw && d < deadzoneRaw) return 0;
  if (d > 0) d -= deadzoneRaw; else d += deadzoneRaw;
  int32_t out = (d * 1000) / (halfRange - deadzoneRaw);
  return (int16_t)clampI32(out, -1000, 1000);
}

// CRC-8/SMBUS (poly 0x07, init 0x00, no reflection, xorout 0x00).
// Check value: crc8("123456789", 9) == 0xF4. Used for CtrlPacket integrity (#4).
static inline uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}
