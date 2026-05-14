#!/usr/bin/env python3
"""joyctl — inject joystick packets over USB serial to the mecanum robot.

Two modes:
  REPL (default):   stream telemetry, send typed commands.
  Sweep (--sweep):  run the base stimulus sequence, write per-test CSV logs,
                    print a summary table.

Pairs with the test-mode block in src/robot/main.cpp (commands t/s/r/x/?).
"""
from __future__ import annotations

import argparse
import csv
import os
import queue
import re
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    import serial  # type: ignore
except ImportError:
    sys.stderr.write("error: pyserial not installed. run: pip install pyserial\n")
    sys.exit(1)


TLM_RE = re.compile(
    r"^TLM\s+ms=(\d+)\s+"
    r"cmd=\[(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\]\s+"
    r"pwm=\[(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]\s+"
    r"raw_tps=\[(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]\s+"
    r"cnt=\[(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\]"
)


@dataclass
class TlmSample:
    ms: int
    cmd: list[int]
    pwm: list[float]
    raw_tps: list[float]
    cnt: list[int]

    def as_csv_row(self) -> list[str]:
        out: list[str] = [str(self.ms)]
        out += [str(v) for v in self.cmd]
        out += [f"{v:.0f}" for v in self.pwm]
        out += [f"{v:.1f}" for v in self.raw_tps]
        out += [str(v) for v in self.cnt]
        return out


def parse_tlm(line: str) -> Optional[TlmSample]:
    m = TLM_RE.match(line.strip())
    if not m:
        return None
    g = m.groups()
    return TlmSample(
        ms=int(g[0]),
        cmd=[int(g[1]), int(g[2]), int(g[3]), int(g[4])],
        pwm=[float(g[5]), float(g[6]), float(g[7]), float(g[8])],
        raw_tps=[float(g[9]), float(g[10]), float(g[11]), float(g[12])],
        cnt=[int(g[13]), int(g[14]), int(g[15]), int(g[16])],
    )


CSV_HEADER = (
    ["ms"]
    + [f"cmd{i}" for i in range(4)]
    + [f"pwm{i}" for i in range(4)]
    + [f"raw_tps{i}" for i in range(4)]
    + [f"cnt{i}" for i in range(4)]
)


class SerialIO:
    """Owns the serial port + a reader thread. Lines flow into self.lines."""

    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.lines: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                self.lines.put(f"<<serial error: {e}>>")
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw = bytes(buf[:nl])
                del buf[: nl + 1]
                try:
                    self.lines.put(raw.decode("utf-8", "replace").rstrip("\r"))
                except Exception:
                    pass

    def send(self, cmd: str) -> None:
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.ser.write(cmd.encode("ascii", "replace"))
        self.ser.flush()

    def close(self) -> None:
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------- REPL ----------------

REPL_HELP = """\
commands:
  t <vx> <vy> <omega>   inject test packet (each -1000..+1000)
  s                     stop (zero packet)
  r                     zero encoders + PID
  x                     exit test mode
  ?                     status
  q                     quit joyctl
  help                  this message
any other text is sent verbatim to the robot.
"""


def repl(sio: SerialIO) -> None:
    sys.stdout.write("joyctl REPL — type 'help' for commands, 'q' to quit\n")
    sys.stdout.flush()
    stop = threading.Event()

    def printer() -> None:
        while not stop.is_set():
            try:
                line = sio.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            sys.stdout.write(line + "\n")
            sys.stdout.flush()

    t = threading.Thread(target=printer, daemon=True)
    t.start()
    try:
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                break
            stripped = line.strip()
            if stripped in ("q", "quit", "exit"):
                break
            if stripped == "help":
                sys.stdout.write(REPL_HELP)
                sys.stdout.flush()
                continue
            if not stripped:
                continue
            sio.send(stripped)
    finally:
        stop.set()


# ---------------- Sweep ----------------

STIMULI = [
    ("vx+", 400, 0, 0),
    ("vx-", -400, 0, 0),
    ("vy+", 0, 400, 0),
    ("vy-", 0, -400, 0),
    ("omega+", 0, 0, 400),
    ("omega-", 0, 0, -400),
]


@dataclass
class StimulusResult:
    label: str
    vx: int
    vy: int
    omega: int
    samples: list[TlmSample] = field(default_factory=list)
    csv_path: Optional[Path] = None

    def steady_state_tps(self, window_s: float = 1.0) -> list[float]:
        if not self.samples:
            return [0.0, 0.0, 0.0, 0.0]
        end = self.samples[-1].ms
        cutoff = end - int(window_s * 1000)
        recent = [s for s in self.samples if s.ms >= cutoff]
        if not recent:
            recent = self.samples[-5:]
        out: list[float] = []
        for i in range(4):
            vals = [s.raw_tps[i] for s in recent]
            out.append(statistics.mean(vals) if vals else 0.0)
        return out


def drain(sio: SerialIO, duration_s: float = 0.0) -> None:
    """Discard everything currently in the queue, then wait duration_s more."""
    deadline = time.monotonic() + duration_s
    while True:
        try:
            sio.lines.get_nowait()
        except queue.Empty:
            if time.monotonic() >= deadline:
                return
            time.sleep(0.01)


def collect(sio: SerialIO, duration_s: float) -> list[TlmSample]:
    samples: list[TlmSample] = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        try:
            line = sio.lines.get(timeout=0.05)
        except queue.Empty:
            continue
        s = parse_tlm(line)
        if s is not None:
            samples.append(s)
    return samples


def sweep(sio: SerialIO, log_dir: Path, capture_s: float = 2.0) -> list[StimulusResult]:
    log_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    results: list[StimulusResult] = []

    # Wake firmware into testMode with a zero packet first so TLM stream starts.
    sio.send("s")
    drain(sio, 0.3)

    for label, vx, vy, w in STIMULI:
        sys.stdout.write(f"-- {label}: t {vx} {vy} {w}\n")
        sys.stdout.flush()

        sio.send("r")
        drain(sio, 0.2)
        sio.send(f"t {vx} {vy} {w}")

        samples = collect(sio, capture_s)
        sio.send("s")

        path = log_dir / f"{ts}-{label}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            w_ = csv.writer(f)
            w_.writerow(CSV_HEADER)
            for s in samples:
                w_.writerow(s.as_csv_row())

        results.append(
            StimulusResult(label=label, vx=vx, vy=vy, omega=w, samples=samples, csv_path=path)
        )
        sys.stdout.write(f"   captured {len(samples)} samples -> {path}\n")
        sys.stdout.flush()
        drain(sio, 0.5)

    sio.send("x")
    drain(sio, 0.2)
    return results


def print_summary(results: list[StimulusResult]) -> None:
    sys.stdout.write("\n=== sweep summary: mean raw_tps in last 1s ===\n")
    sys.stdout.write(f"{'stimulus':<8} {'cmd':<22} {'slot0':>9} {'slot1':>9} {'slot2':>9} {'slot3':>9}\n")
    for r in results:
        tps = r.steady_state_tps()
        cmd_s = f"({r.vx:+d},{r.vy:+d},{r.omega:+d})"
        sys.stdout.write(
            f"{r.label:<8} {cmd_s:<22} {tps[0]:>9.1f} {tps[1]:>9.1f} {tps[2]:>9.1f} {tps[3]:>9.1f}\n"
        )
    sys.stdout.write("\nFill in visual observations per wheel (FL/FR/RL/RR direction)\n")
    sys.stdout.write("for each stimulus, then cross-reference with this table.\n")
    sys.stdout.flush()


# ---------------- main ----------------

def main() -> int:
    p = argparse.ArgumentParser(description="Inject joystick packets to mecanum robot over USB serial.")
    p.add_argument("--port", default="COM8", help="serial port (default COM8)")
    p.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    p.add_argument("--sweep", action="store_true", help="run base stimulus sweep instead of REPL")
    p.add_argument("--capture-seconds", type=float, default=2.0, help="per-stimulus capture window (sweep)")
    p.add_argument("--log-dir", default=None, help="output dir for sweep CSVs (default tools/logs/)")
    args = p.parse_args()

    log_dir = Path(args.log_dir) if args.log_dir else Path(__file__).resolve().parent / "logs"

    try:
        sio = SerialIO(args.port, args.baud)
    except serial.SerialException as e:
        sys.stderr.write(f"error: cannot open {args.port}: {e}\n")
        return 2

    try:
        if args.sweep:
            results = sweep(sio, log_dir, capture_s=args.capture_seconds)
            print_summary(results)
        else:
            repl(sio)
    finally:
        sio.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
