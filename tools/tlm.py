"""Parser for the robot's serial telemetry, shared by every bench tool.

The firmware prints `key=value` and `key=[a b c d]` fields (emitTlm and the 2 Hz
status line in src/robot/main.cpp). Fields are looked up by NAME, so adding or
reordering a field in the firmware does not break the tools.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

_FIELD_RE = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")


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
