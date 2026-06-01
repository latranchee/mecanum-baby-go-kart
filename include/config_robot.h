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
static const float MAX_TPS = 2100.0f;                   // ticks/sec at full PWM (ramp test)
static const float Kff     = (float)PWM_MAX / MAX_TPS;  // ~0.487 feed-forward gain
static const float Kp      = 0.15f;
static const float Ki      = 0.4f;                      // conditional-integration anti-windup
static const float I_MAX   = 0.6f * (float)PWM_MAX;     // bound integral authority (~614)

// Command slew limiter (cmd units/sec). Staggers inrush current on 0->full.
static const float CMD_SLEW = 4000.0f;

// Link watchdog: stop motors if no fresh packet for this long.
static const uint32_t WATCHDOG_MS = 500;
