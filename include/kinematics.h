#pragma once
#include <stdint.h>

// Mecanum wheel mixing — pure integer math, no Arduino deps (host-testable).
// Wheel command in [-1000..+1000] (thousandths of max wheel speed).
// X-pattern rollers, slot map: M1=FL, M2=FR, M3=RL, M4=RR.
static inline void mecanumMix(int16_t vx, int16_t vy, int16_t omega, int32_t outCmd[4]) {
  outCmd[0] = (int32_t)vx - vy - omega;  // FL
  outCmd[1] = (int32_t)vx + vy + omega;  // FR
  outCmd[2] = (int32_t)vx + vy - omega;  // RL
  outCmd[3] = (int32_t)vx - vy + omega;  // RR

  int32_t peak = 1000;
  for (int i = 0; i < 4; i++) {
    int32_t a = outCmd[i] < 0 ? -outCmd[i] : outCmd[i];
    if (a > peak) peak = a;
  }
  if (peak > 1000) {
    for (int i = 0; i < 4; i++) outCmd[i] = (outCmd[i] * 1000) / peak;
  }
}
