"""lab_analysis.py  --  numbers from logs.  Pure numpy, no ROS."""
from __future__ import annotations

import numpy as np


def staircase_sequence(amplitude: float, step: float) -> list[tuple[float, str]]:
    """Command sequence for a hysteresis staircase: 0 -> +A -> -A -> 0.

    Returns [(offset, phase)], phase in {"asc1", "desc", "asc2"}.
    """
    if step <= 0 or amplitude <= 0:
        raise ValueError("amplitude and step must be > 0")
    n = int(round(amplitude / step))
    seq = [(i * step, "asc1") for i in range(0, n + 1)]
    seq += [(amplitude - i * step, "desc") for i in range(1, 2 * n + 1)]
    seq += [(-amplitude + i * step, "asc2") for i in range(1, n + 1)]
    return seq


def analyse_staircase(cmd, meas, phase, noise_floor: float) -> dict:
    """Dead band / hysteresis numbers from a staircase test.

    cmd, meas   commanded and measured offset (same unit), one value per step
    phase       "asc1" / "desc" / "asc2" per step
    noise_floor smallest change in `meas` counted as motion (same unit)
    """
    cmd, meas = np.asarray(cmd, float), np.asarray(meas, float)
    phase = np.asarray(phase)
    direction = np.where(phase == "desc", -1.0, 1.0)
    lag = (cmd - meas) * direction        # >0: measurement is behind the command

    out: dict = {"max_lag": float(np.max(lag)), "lag_per_step": lag}

    # Breakaway: first step of each leg where the measurement has moved more than
    # the noise floor away from where the leg started.
    breakaway = {}
    for leg in ("asc1", "desc", "asc2"):
        idx = np.flatnonzero(phase == leg)
        if len(idx) < 2:
            continue
        start_meas = meas[idx[0] - 1] if idx[0] > 0 else meas[idx[0]]
        start_cmd = cmd[idx[0] - 1] if idx[0] > 0 else cmd[idx[0]]
        moved = np.flatnonzero(np.abs(meas[idx] - start_meas) > noise_floor)
        breakaway[leg] = (None if len(moved) == 0
                          else float(abs(cmd[idx[moved[0]]] - start_cmd)))
    out["breakaway_cmd_travel"] = breakaway

    # Steady lag once moving: median over the second half of each leg.
    steady = []
    for leg in ("asc1", "desc", "asc2"):
        idx = np.flatnonzero(phase == leg)
        if len(idx) >= 4:
            steady.append(np.median(lag[idx[len(idx) // 2:]]))
    out["steady_lag"] = float(np.median(steady)) if steady else float("nan")

    # Hysteresis width: measured value on the way down minus on the way up at the same
    # commanded offset (only where both exist).
    up_mask = (phase == "asc1") | (phase == "asc2")
    dn_mask = phase == "desc"
    widths = []
    for c, m in zip(cmd[dn_mask], meas[dn_mask]):
        j = np.flatnonzero(up_mask & (np.abs(cmd - c) < 1e-9 + 1e-6 * max(1.0, abs(c))))
        if len(j):
            widths.append(m - float(np.mean(meas[j])))
    out["hysteresis_width"] = float(np.mean(widths)) if widths else float("nan")
    return out


def rms(x) -> float:
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


def drift_rate(t, y, last_fraction: float = 0.5) -> float:
    """Slope (units/s) of a straight-line fit over the last `last_fraction` of the data."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    n = len(t)
    if n < 3:
        return float("nan")
    i0 = int(n * (1 - last_fraction))
    return float(np.polyfit(t[i0:], y[i0:], 1)[0])
