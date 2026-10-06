# Pinout

## Driver Module (per board, 2 channels)

| Label | Schema | Role |
|---|---|---|
| **V** | VCC | Logic supply 3–5V |
| **B1/B2** | INBx | Direction input B (channel 1/2) |
| **A1/A2** | INAx | Direction input A (channel 1/2) |
| **P1/P2** | PWMx | PWM speed (channel 1/2) |
| **G** | GND | Logic ground |

Two V and two G pins are redundant rails for wiring convenience (tie both to same 5V/GND).

Per motor: P (speed) + A + B (direction state) = 3 signal wires from MCU.

### Direction truth table

| INA | INB | PWM | Result |
|---|---|---|---|
| H | L | PWM | Forward (speed = PWM duty) |
| L | H | PWM | Reverse |
| L | L | x | Brake/coast |
| H | H | x | Brake (short) |

## Encoder (per motor, 6 wires)

| Wire | Role |
|---|---|
| M+ | Motor power positive (to driver output) |
| M- | Motor power negative (to driver output) |
| VCC | Encoder sensor supply (3.3V or 5V) |
| GND | Encoder sensor ground |
| A | Quadrature phase A (to MCU, PCNT input) |
| B | Quadrature phase B (to MCU, PCNT input) |

**Warning:** VCC and GND polarity must be correct or encoder PCB will be damaged.

## GPIO Budget (ESP32)

| Use | Pins | Type |
|---|---|---|
| Driver PWM (4 channels) | 4 | PWM-capable |
| Driver INA (4 channels) | 4 | GPIO |
| Driver INB (4 channels) | 4 | GPIO |
| Encoder A (4 motors) | 4 | Any input (PCNT, input-only OK) |
| Encoder B (4 motors) | 4 | Any input (PCNT, input-only OK) |
| **Total** | **20** | 4 PWM |

---

## Current wiring (1 ESP32, 2 driver modules, 4 motors)

Source of truth: the `motors[]` table at the top of `src/robot/main.cpp`. Slot =
array index = wheel corner. Encoders are decoded in hardware by the PCNT pulse
counters (4x quadrature), so any input-capable GPIO works for A and B.

| Slot | Firmware corner | Rider's corner | PWM | INA | INB | Enc A | Enc B | Harness labels (layout table below) |
|---|---|---|---|---|---|---|---|---|
| 0 | FL | **RR** | 18 | 5 | 19 | 13 | 17 | P3, A3, B3 / M4 enc |
| 1 | FR | **RL** | 21 | 23 | 22 | 16 | 4 | P4, B4 = INA, A4 = INB / M3 enc |
| 2 | RL | **FR** | 25 | 27 | 33 | 39 | 36 | P1, B1 = INA, A1 = INB / M1 enc |
| 3 | RR | **FL** | 26 | 32 | 14 | 35 | 34 | P2, A2, B2 / M2 enc |

The rider faces the firmware's rear (the controller inverts vx and vy; see the
README "Wheel layout"), so the firmware's FL is the rider's rear-right:
bench-confirmed 2026-10-05 by spinning slot 0 alone.

The harness numbers (1-4, M1-M4) are the physical driver channels and wire
labels; they predate the 2026-05-31 body swap. INA/INB in this table are what
the code drives, and they do not always match the A/B wire label (slots 1 and 2
use the "B" wire as INA). The direction each slot gets for +PWM was set by
solo-PWM observation on the bench, not derived from the labels: `motors[]` in
`src/robot/main.cpp` is authoritative.

### Boot and reset state

Between reset and `setup()` every GPIO is an input, so the driver's PWM and
direction inputs float. GPIO5 (FL INA) and GPIO14 (RR INB) are also commonly
reported to toggle during the ROM boot. Each driver PWM input (GPIO18, 21, 25,
26) should have a pull-down (10 kOhm to GND) unless the driver module already
has one, so a reset, a brownout or a watchdog reboot can't twitch a wheel.
Check the module before relying on it.

The loop watchdog's ISR hook drives every INA/INB LOW (brake/coast) before the
reboot, which covers the hang itself; the pull-downs cover the reboot window.

### No free ADC pin

ADC1 is GPIO32-39 and all of them are used (32/33 direction, 34/35/36/39
encoders). ADC2 cannot be read while ESP-NOW runs. Battery sensing needs a
direction line moved off GPIO32/33, or an I2C monitor (INA226: voltage and
motor current).

### ESP32-WROOM-32D 38-pin layout

USB at bottom. Left/right columns mirror physical board edges. **Bold** = wired now.

| Left pin | Use | &nbsp; | Right pin | Use |
|---|---|---|---|---|
| 3V3 | — | | GND | — |
| EN | — | | GPIO23 | **B4 ← purple** |
| GPIO36 (SVP) | **M1 enc B ← white** | | GPIO22 | **A4 ← yellow** |
| GPIO39 (SVN) | **M1 enc A ← yellow** | | GPIO1 (TX) | USB serial |
| GPIO34 | **M2 enc B ← white** | | GPIO3 (RX) | USB serial |
| GPIO35 | **M2 enc A ← yellow** | | GPIO21 | **P4 ← brown** |
| **GPIO32** | **A2 ← yellow** | | GND | — |
| **GPIO33** | **A1 ← green** | | GPIO19 | **B3 ← gray** |
| **GPIO25** | **P1 ← blue** | | GPIO18 | **P3 ← blue** |
| **GPIO26** | **P2 ← brown** | | **GPIO5** | **A3 ← green** |
| **GPIO27** | **B1 ← gray** | | **GPIO17** | **M4 enc B ← white** |
| **GPIO14** | **B2 ← purple** | | **GPIO16** | **M3 enc A ← yellow** |
| GPIO12 | strapping — avoid | | **GPIO4** | **M3 enc B ← white** |
| **GND** | **G ← black** | | GPIO0 | strapping — avoid |
| GPIO13 | **M4 enc A ← yellow** | | GPIO2 | strapping — avoid |
| GPIO9 (SD2) | flash — DO NOT USE | | GPIO15 | strapping — avoid |
| GPIO10 (SD3) | flash — DO NOT USE | | GPIO8 (SD1) | flash — DO NOT USE |
| GPIO11 (CMD) | flash — DO NOT USE | | GPIO7 (SD0) | flash — DO NOT USE |
| **5V** | **V ← red** | | GPIO6 (CLK) | flash — DO NOT USE |
