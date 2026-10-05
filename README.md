# mecanum

Four-wheel mecanum robot driven over a wireless ESP-NOW link. Two ESP32 firmware
targets in one PlatformIO project, sharing a single packet definition
(`include/protocol.h`):

- **robot** (`src/robot/main.cpp`, ESP32-WROOM `esp32dev`) — receives control
  packets, runs mecanum kinematics + per-wheel PI velocity control, drives 4 DC
  motors through dual H-bridge driver modules with quadrature encoder feedback.
- **controller** (`src/controller/main.cpp`, M5 AtomS3) — reads the M5 Atom
  JoyStick, shapes the sticks through a response curve, and transmits
  `CtrlPacket`s at ~50 Hz with an LCD status display.
- **headset** (`src/headset/`, Waveshare ESP32-S3-LCD-1.69) — optional head-tilt
  transmitter, a drop-in alternative to the controller (same packet/MAC/channel).

The link sends a packed `CtrlPacket {seq, vx, vy, omega, buttons, flags, crc}` at
50 Hz. The robot drops frames with a bad CRC8, follows one transmitter at a time
(frames from a second one are dropped while the link is live), accepts only a newer
seq from it, and stops the motors if no fresh packet arrives within 500 ms
(watchdog) or an e-stop flag is set. After 500 ms of silence it accepts the next
valid frame from any transmitter, so a rebooted controller is picked up at once.

## Safety behaviour

| Event | Robot response |
|---|---|
| No valid packet for 500 ms | Motors stop (link watchdog) |
| E-stop flag | Motors stop and stay stopped while the flag is sent |
| A wheel at ≥30% PWM with no encoder motion for 300 ms (dead encoder, jammed wheel) | **Drive fault**: all motors stop, `FAULT FL ...` on serial. Clears after 1 s of neutral sticks, or `r`/`x` on the bench |
| `loop()` hangs for 500 ms | Task watchdog: every direction pin driven LOW, then reboot |
| Bench test: no serial line for 1 s while driving | Test stops (`k` is the keepalive) |
| Bench test: radio e-stop | Test mode aborted, motors stopped |

## Controller

The AtomS3 status bar shows one word, highest priority first:

| Status | Meaning |
|---|---|
| `NO RADIO` | ESP-NOW init or peer add failed; nothing is sent |
| `JOY ERR` | Joystick (I2C) not answering for 100 ms; e-stop frames sent, bus re-initialized every 500 ms |
| `E-STOP` | Both stick clicks held |
| `CENTER` | Not armed: release and centre both sticks |
| `ONLINE` / `OFFLINE` | Armed; the robot's radio is / isn't acknowledging |

- **Stick centre** is taken from 16 readings at rest near mid-scale. A stick held at
  power-on is refused (the controller keeps trying until the sticks are released).
- **Arming**: the controller sends drive commands only when armed. It starts
  disarmed; the e-stop and a joystick fault disarm it. It re-arms when the e-stop is
  released and every stick is centred. While disarmed it sends the e-stop flag, so
  letting go of the clicks with a stick still pushed doesn't launch the cart.
- **Buttons**: yellow L/R = speed −/+ 10%. Left stick click cycles the mode
  (FULL → NO GOV → NO BODY → OPEN LP → RAW) **on release**, and only if the right
  click wasn't pressed during it. Both stick clicks = e-stop.

## Wheel layout

Top-down, robot facing forward. Slot index = array index in `motors[]`:

```
   M1 (FL) ---- M2 (FR)
      |            |
   M3 (RL) ---- M4 (RR)
```

X-pattern rollers. `mecanumMix(vx, vy, omega)` (`include/kinematics.h`):

| Wheel | mix |
|-------|-----|
| FL | vx − vy − omega |
| FR | vx + vy + omega |
| RL | vx + vy − omega |
| RR | vx − vy + omega |

`vx` forward+, `vy` strafe-right+, `omega` CCW+, each in [−1000, +1000]. Output is
scaled down if any wheel saturates past ±1000.

## Hardware / wiring

Full pin map, driver truth table, and the ESP32 38-pin layout are in
[`docs/pinout.md`](docs/pinout.md). Motor/encoder slot mapping, corner swaps, and
encoder sign conventions are documented inline at the top of
`src/robot/main.cpp`.

## Build & flash

Requires [PlatformIO](https://platformio.org/) (`pio`).

```sh
pio run -e robot                    # build robot
pio run -e robot -t upload          # flash robot       (upload_port COM8)
pio run -e controller -t upload     # flash controller  (upload_port COM6)
pio run -e headset -t upload        # flash headset      (upload_port COM7)
pio device monitor -e robot         # serial @ 115200
```

Adjust the `upload_port`/`monitor_port` in `platformio.ini` to match your COM
ports. `pio run` with no `-e` builds the default `robot` env.

> **CRC wire break:** the `crc` byte made `CtrlPacket` 13 bytes. The robot rejects
> any frame that isn't exactly `sizeof(CtrlPacket)`, so after pulling these
> changes **flash both the robot and a transmitter** — a mismatched pair won't
> talk.

## secrets.h setup

The robot MAC and ESP-NOW keys live in `include/secrets.h`, which is gitignored.
`protocol.h` includes it automatically when present (`#if __has_include`), else
falls back to a built-in MAC so a fresh clone still builds.

```sh
cp include/secrets.h.example include/secrets.h
# edit secrets.h: set ROBOT_MAC (from the robot's "Robot MAC: ..." boot log)
```

`CONTROLLER_MAC`, `ESPNOW_PMK`, and `ESPNOW_LMK` are only needed if you turn on
link encryption (below).

## Test mode (robot serial console)

When the robot sees a `t`, `m` or `s` command it enters test mode and ignores
ESP-NOW drive commands. A radio **e-stop** still works: it aborts test mode.
Send commands over the 115200 serial console (or via `tools/joyctl.py`):

| Cmd | Effect |
|-----|--------|
| `t <vx> <vy> <omega>` | mix path: kinematics → PID → motors (each −1000..+1000) |
| `m <slot> <pwm>` | direct path: raw PWM on one slot (slot 0..3, pwm −1023..+1023) |
| `s` | stop (zero everything) |
| `r` | zero encoder counters + reset PID, clear a drive fault |
| `x` | exit test mode (ESP-NOW control resumes), clear a drive fault |
| `k` | keepalive (silent) |
| `?` | print one-shot status |

A test that drives stops if no serial line arrives for 1 s. The Python tools send
`k` every 250 ms and stop the robot on every exit path, so a crashed script or a
pulled cable stops the cart. Typing commands by hand in a plain serial monitor,
resend within the second or the test stops.

In test mode the robot streams `TLM ...` lines (cmd / pwm / raw_tps / encoder
counts / fault mask) at 20 Hz; otherwise it prints a `seq=... crcDrops=...` status
line at 2 Hz. `cmd=` is what the wheel loops were actually given (after slew,
governor and body correction).

## tools/

Python helpers (need `pyserial`: `pip install pyserial`). Each takes the robot's
COM port as an optional first arg.

| Tool | Purpose |
|------|---------|
| `joyctl.py [PORT]` | REPL to stream telemetry + send `t/s/r/x/?`; `--sweep` runs a stimulus sequence, logs per-test CSVs, prints a summary |
| `verify_sweep.py` | check the sweep CSVs against expected per-wheel encoder sign/magnitude (reads `encSign`/`MAX_TPS` from the firmware) |
| `solo_test.py [PORT]` | drive each slot in isolation (direct PWM), capture mean `raw_tps` per slot |
| `spin_fr.py [PORT]` | spin slot 1 (FR) and stream its `raw_tps` live (encoder-connector debugging) |
| `calibrate_maxtps.py` | measure each wheel's full-PWM tick rate for `MAX_TPS[]` (wheels off the ground) |
| `drive_watch.py` | drive a stimulus and flag a mid-run reboot (brownout hunting) |

## Host unit tests

The pure logic headers (`kinematics.h`, `governor.h`, `body_loop.h`, `curve.h`,
`control_math.h`, `safety.h`, `controller_logic.h`) compile natively and are
covered by Unity tests: `test/test_math` for the sign / remap / scaling / control
math, `test/test_safety` for the link gate, drive fault, stick calibration, arming
and the mode-click gesture.

```sh
pio test -e native
```

Needs a host C++ compiler (`g++`/`clang++`) on `PATH` — the ESP toolchain is not
enough. On Windows, install MinGW-w64 (e.g. `winget install BrechtSanders.WinLibs.POSIX.UCRT`)
and ensure its `mingw64\bin` is on `PATH`.

## Optional: ESP-NOW encryption

Off by default (`ESPNOW_ENCRYPT 0` in `protocol.h`). To enable:

1. Fill `ESPNOW_PMK`, `ESPNOW_LMK` (16 bytes each) and `CONTROLLER_MAC` in
   `secrets.h` — identical on both ends.
2. Build both with the flag set, e.g. `PLATFORMIO_BUILD_FLAGS="-DESPNOW_ENCRYPT=1" pio run -e robot -e controller -t upload`, or flip the default in `protocol.h`.
3. Flash robot and transmitter back-to-back from the same `secrets.h`, and
   confirm **ONLINE** before walking away — mismatched keys silently kill the
   link. Roll back by setting the flag to 0 and reflashing.

## Optional: battery telemetry

No sensing is wired by default (`BATT_ADC_PIN = -1` in `config_robot.h`), so no
voltage is logged. The robot always logs its `reset reason` at boot (a
`BROWNOUT` there means the supply rail collapsed). On the current pin map no ADC
pin is free: ADC1 (GPIO32-39) is fully used and ADC2 is unavailable while ESP-NOW
runs (see `docs/pinout.md`). After freeing an ADC1 pin, set `BATT_ADC_PIN` to it
and `BATT_DIVIDER` to the divider ratio; `vbat=` then appears in the log lines.
