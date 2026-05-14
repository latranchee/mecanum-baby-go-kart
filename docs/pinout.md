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
| A | Quadrature phase A (to MCU, interrupt pin) |
| B | Quadrature phase B (to MCU, GPIO) |

**Warning:** VCC and GND polarity must be correct or encoder PCB will be damaged.

## GPIO Budget (ESP32)

| Use | Pins | Type |
|---|---|---|
| Driver PWM (4 channels) | 4 | PWM-capable |
| Driver INA (4 channels) | 4 | GPIO |
| Driver INB (4 channels) | 4 | GPIO |
| Encoder A (4 motors) | 4 | Interrupt-capable |
| Encoder B (4 motors) | 4 | GPIO (input-only OK) |
| **Total** | **20** | 4 PWM + 4 INT |

---

## Current wiring (1 ESP32 + 1 driver module, 2 motors)

### Driver → ESP32 wire map

| Driver pin | Wire color | ESP32 pin | Role |
|---|---|---|---|
| G | black | GND | Logic ground |
| G | — | — | (unused, redundant) |
| V | red | 5V | Logic supply |
| V | — | — | (unused, redundant) |
| B1 | gray | GPIO27 | Motor 1 INB |
| B2 | purple | GPIO14 | Motor 2 INB |
| A1 | green | GPIO33 | Motor 1 INA |
| A2 | yellow | GPIO32 | Motor 2 INA |
| P1 | blue | GPIO25 | Motor 1 PWM |
| P2 | brown | GPIO26 | Motor 2 PWM |

### Encoder → ESP32 wire map

| Motor | Phase | Wire color | ESP32 pin |
|---|---|---|---|
| M1 | A | yellow | GPIO16 |
| M1 | B | white | GPIO17 |
| M2 | A | yellow | GPIO4 |
| M2 | B | white | GPIO5 |

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
