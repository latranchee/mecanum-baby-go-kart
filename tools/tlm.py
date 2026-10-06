"""Parser for the robot's serial telemetry, shared by every bench tool.

The firmware prints `key=value` and `key=[a b c d]` fields (emitTlm and the 2 Hz
status line in src/robot/main.cpp). Fields are looked up by NAME, so adding or
reordering a field in the firmware does not break the tools.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

_FIELD_RE = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")

_REPO = Path(__file__).resolve().parent.parent

# Motor slots are named in the firmware frame (FL FR RL RR), but the rider faces
# the firmware's rear (controller INVERT_VX/VY; bench-confirmed 2026-10-05: slot 0
# is the rider's rear-right). Tools print the rider's names for humans.
FIRMWARE_NAME = ["FL", "FR", "RL", "RR"]
RIDER_NAME = ["RR", "RL", "FR", "FL"]

# Bench tools send this while a test drives: the robot stops a driving test after
# TEST_LINK_MS (1 s) with no serial line, so a crashed script or pulled cable can't
# leave it running. The robot accepts `k` silently.
KEEPALIVE_S = 0.25


def _c_floats(text: str, pattern: str) -> list[float]:
    m = re.search(pattern, text)
    if not m:
        raise ValueError(f"pattern not found: {pattern}")
    return [float(v.strip().rstrip("fF")) for v in m.group(1).split(",") if v.strip()]


@dataclass
class RobotConfig:
    max_tps: list[float]      # include/config_robot.h MAX_TPS[4], slot order FL FR RL RR
    speed_ref_frac: float     # include/config_robot.h SPEED_REF_FRAC
    enc_sign: list[int]       # src/robot/main.cpp encSign[4]

    @property
    def ref_tps(self) -> float:
        """Ticks/sec that cmd 1000 targets on every wheel (firmware cmdRefTps())."""
        return self.speed_ref_frac * min(self.max_tps)


def robot_config() -> RobotConfig:
    """Read the calibration constants straight from the firmware sources, so the
    tools can't drift from what the robot runs."""
    cfg = (_REPO / "include" / "config_robot.h").read_text(encoding="utf-8")
    src = (_REPO / "src" / "robot" / "main.cpp").read_text(encoding="utf-8")
    return RobotConfig(
        max_tps=_c_floats(cfg, r"MAX_TPS\[4\]\s*=\s*\{([^}]*)\}"),
        speed_ref_frac=_c_floats(cfg, r"SPEED_REF_FRAC\s*=\s*([0-9.]+f?)\s*;")[0],
        enc_sign=[int(v) for v in _c_floats(src, r"encSign\[4\]\s*=\s*\{([^}]*)\}")],
    )


def parse_fields(line: str) -> dict[str, str]:
    """Every `key=value` / `key=[...]` field on the line, values left raw."""
    return dict(_FIELD_RE.findall(line))


def _quad(raw: str, cast):
    vals = [cast(v) for v in raw.strip("[]").split()]
    if len(vals) != 4:
        raise ValueError(f"expected 4 values, got {len(vals)}")
    return vals


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


CSV_HEADER = (
    ["ms"]
    + [f"cmd{i}" for i in range(4)]
    + [f"pwm{i}" for i in range(4)]
    + [f"raw_tps{i}" for i in range(4)]
    + [f"cnt{i}" for i in range(4)]
)


def parse_tlm(line: str) -> Optional[TlmSample]:
    """A `TLM ...` line as a TlmSample; None for any other or truncated line."""
    line = line.strip()
    if not line.startswith("TLM "):
        return None
    f = parse_fields(line)
    try:
        return TlmSample(
            ms=int(f["ms"]),
            cmd=_quad(f["cmd"], int),
            pwm=_quad(f["pwm"], float),
            raw_tps=_quad(f["raw_tps"], float),
            cnt=_quad(f["cnt"], int),
        )
    except (KeyError, ValueError):
        return None
