#!/usr/bin/env python3
"""calibrate_maxtps — measure each wheel's full-PWM tick rate for MAX_TPS[4].

Drives ONE wheel at a time at full PWM (direct mode, bypassing kinematics + PID),
reads the steady-state |raw_tps| from the robot's TLM stream, and prints a ready-to
-paste MAX_TPS[4] line for include/config_robot.h. Runs both directions and warns
on >15% fwd/rev asymmetry (a sign of a weak driver/encoder/supply on that wheel).

SAFETY: full PWM spins the wheels fast. PUT THE WHEELS OFF THE GROUND first or the
cart will run away. Each wheel runs ~`--spin` seconds.

Pairs with the test-mode block in src/robot/main.cpp (commands m/s/r/x).
"""
from __future__ import annotations

import argparse
import re
import statistics
import sys
import time

try:
    import serial  # type: ignore
except ImportError:
    sys.stderr.write("error: pyserial not installed. run: pip install pyserial\n")
    sys.exit(1)

# TLM lines carry extra fields (gov=, batt) in varying positions, so anchor only
# on raw_tps[] and search rather than match the whole line.
TLM_RE = re.compile(
    r"raw_tps=\[(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]"
)

SLOTS = ["FL", "FR", "RL", "RR"]


def read_lines(ser, seconds):
    """Yield decoded lines from the port for `seconds`."""
    buf = bytearray()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        buf.extend(chunk)
        while True:
            nl = buf.find(b"\n")
            if nl < 0:
                break
            line = bytes(buf[: nl]).decode("utf-8", "replace").rstrip("\r")
            del buf[: nl + 1]
            yield line


def measure(ser, slot, pwm, spin_s, settle_s):
    """Drive `slot` at `pwm`, return mean |raw_tps[slot]| over the steady window."""
    ser.write(f"r\n".encode()); ser.flush(); time.sleep(0.2)
    ser.write(f"m {slot} {pwm}\n".encode()); ser.flush()
    samples = []
    t0 = time.monotonic()
    for line in read_lines(ser, spin_s):
        m = TLM_RE.search(line.strip())
        if not m:
            continue
        if time.monotonic() - t0 < settle_s:   # skip spin-up transient
            continue
        samples.append(abs(float(m.group(slot + 1))))
    ser.write(b"s\n"); ser.flush(); time.sleep(0.4)
    if not samples:
        return 0.0, 0
    return statistics.mean(samples), len(samples)


def main() -> int:
    p = argparse.ArgumentParser(description="Measure per-wheel full-PWM tick rate.")
    p.add_argument("--port", default="COM8")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--pwm", type=int, default=1023, help="drive PWM (default full 1023)")
    p.add_argument("--spin", type=float, default=2.0, help="seconds per direction")
    p.add_argument("--settle", type=float, default=0.8, help="ignore first N s (spin-up)")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.stderr.write(f"error: cannot open {args.port}: {e}\n")
        return 2

    time.sleep(1.8)                          # RTS resets the ESP32 on open; wait for boot
    ser.reset_input_buffer()
    ser.write(b"s\n"); ser.flush()           # enter test mode, all stopped
    time.sleep(0.3)
    # Confirm we're actually in test mode (TLM streaming) before driving.
    if not any(TLM_RE.search(l.strip()) for l in read_lines(ser, 1.0)):
        ser.write(b"s\n"); ser.flush(); time.sleep(0.3)
        if not any(TLM_RE.search(l.strip()) for l in read_lines(ser, 1.0)):
            sys.stderr.write("error: no TLM stream — robot not in test mode. Check port/boot.\n")
            ser.close(); return 3

    print("=== MAX_TPS calibration (wheels MUST be off the ground) ===")
    print(f"port={args.port} pwm={args.pwm} spin={args.spin}s/dir\n")
    fwd, rev = [0.0] * 4, [0.0] * 4
    for s in range(4):
        f, nf = measure(ser, s, args.pwm, args.spin, args.settle)
        r, nr = measure(ser, s, -args.pwm, args.spin, args.settle)
        fwd[s], rev[s] = f, r
        lo, hi = min(f, r), max(f, r)
        skew = 0.0 if hi == 0 else (hi - lo) / hi * 100.0
        warn = "  <-- WARN >15% fwd/rev skew" if skew > 15 else ""
        print(f"  {SLOTS[s]} (slot {s}): fwd={f:7.0f} ({nf:3d})  rev={r:7.0f} ({nr:3d})  skew={skew:4.1f}%{warn}")

    ser.write(b"x\n"); ser.flush()           # exit test mode -> ESP-NOW resumes
    ser.close()

    # Use the smaller of fwd/rev per wheel: it's the rate that limits closed-loop.
    maxtps = [min(fwd[i], rev[i]) for i in range(4)]
    print("\nPaste into include/config_robot.h (slot order [FL,FR,RL,RR]):")
    print(f"static const float MAX_TPS[4] = {{ {maxtps[0]:.0f}.0f, {maxtps[1]:.0f}.0f, "
          f"{maxtps[2]:.0f}.0f, {maxtps[3]:.0f}.0f }};")
    weakest = min(maxtps)
    print(f"\nWeakest wheel (the uniform target ref) = {weakest:.0f} tps"
          f" ({SLOTS[maxtps.index(weakest)]}).")
    if weakest > 0 and max(maxtps) / weakest > 1.2:
        print("NOTE: >20% spread across wheels — the two battery halves are mismatched."
              "\n      Top speed is capped to the weakest. Match pack V/SoC to raise it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
