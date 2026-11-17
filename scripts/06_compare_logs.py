#!/usr/bin/env python3
"""06_compare_logs.py  --  put a sim log and a real log side by side.   (offline: no ROS)

Works on the CSV files the other scripts write into data/logs/ (inside the container: /home/ros/data/logs):

  * trajectory logs (hold / waypoints / geometry): compares tracking error, joint
    positions and torques.  Time is aligned to the start of each log; use --tag to
    compare only one phase (e.g. --tag trace for 03_geometry.py).
  * *_steps.csv files (dead-band tests): prints dead-band numbers side by side and
    overlays the hysteresis loops.

  ./fr3 cmp circle_k300_15 --tag trace           # finds the latest sim and real log of that name and calls this script
  ./fr3 run fake 06_compare_logs.py /home/ros/data/logs/circle_sim_2026....csv /home/ros/data/logs/circle_real_2026....csv --png /home/ros/data/logs/out.png

Reading the numbers: tracking RMS/max error tells you how well each plant follows the SAME
target; if real is worse in a consistent way, tune the sim (friction/damping/armature in
the model) until the numbers and curves line up.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from lab_analysis import analyse_staircase, rms


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return rows


def label(path: str) -> str:
    p = Path(path)
    meta = Path(str(path).replace("_steps.csv", ".json").replace(".csv", ".json"))
    mode = "?"
    if meta.exists():
        try:
            mode = json.loads(meta.read_text()).get("mode", "?")
        except Exception:  # noqa: BLE001
            pass
    for m in ("sim", "real"):
        if f"_{m}_" in p.name:
            mode = m
    return f"{mode}:{p.stem}"


def compare_steps(paths, png):
    data = []
    for p in paths:
        rows = read_csv(p)
        cmd = np.array([float(r["cmd"]) for r in rows])
        meas = np.array([float(r["meas"]) for r in rows])
        ph = np.array([r["phase"] for r in rows])
        unit = rows[0]["unit"]
        step = float(np.median(np.abs(np.diff(cmd))))
        res = analyse_staircase(cmd, meas, ph, noise_floor=0.02 * step)
        data.append((label(p), cmd, meas, ph, res, unit))
    print(f"{'':30s} " + " ".join(f"{d[0][:28]:>30s}" for d in data))
    for key, nice in (("max_lag", "max lag"), ("steady_lag", "steady lag (half dead band)"),
                      ("hysteresis_width", "hysteresis width")):
        print(f"{nice:30s} " + " ".join(f"{d[4][key]:>26.4f} {d[5]:>3s}" for d in data))
    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 6))
        for name, cmd, meas, ph, _, unit in data:
            ax.plot(cmd, meas, "o-", ms=3, lw=1, label=name)
        lim = max(np.max(np.abs(data[0][1])), 1e-9)
        ax.plot([-lim, lim], [-lim, lim], "k:", lw=0.8, label="ideal")
        ax.set_xlabel(f"commanded offset [{data[0][5]}]"), ax.set_ylabel("measured offset")
        ax.grid(True), ax.legend(fontsize=7), ax.set_title("hysteresis")
        fig.savefig(png, dpi=130, bbox_inches="tight")
        print("wrote", png)


def load_traj(path, tag):
    rows = read_csv(path)
    if tag:
        rows = [r for r in rows if r["tag"].startswith(tag)]
    if not rows:
        raise SystemExit(f"{path}: no rows{' with tag ' + tag if tag else ''}")
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != "tag"}
    cols["t"] = cols["t"] - cols["t"][0]
    return cols


def compare_traj(paths, tag, png):
    logs = [(label(p), load_traj(p, tag)) for p in paths]
    print(f"{'':30s} " + " ".join(f"{n[:28]:>30s}" for n, _ in logs))
    rows = [
        ("duration [s]", lambda c: c["t"][-1]),
        ("position error RMS [mm]", lambda c: rms(c["pos_err_mm"])),
        ("position error max [mm]", lambda c: np.nanmax(c["pos_err_mm"])),
        ("orientation error RMS [deg]", lambda c: rms(c["ang_err_deg"])),
        ("orientation error max [deg]", lambda c: np.nanmax(c["ang_err_deg"])),
        ("hand angle from down, mean [deg]", lambda c: np.nanmean(c["down_angle_deg"])),
    ] + [(f"|torque| mean joint {i} [Nm]", (lambda i: lambda c: np.nanmean(np.abs(c[f"tau{i}"])))(i))
         for i in range(1, 8)]
    for nice, fn in rows:
        vals = []
        for _, c in logs:
            try:
                vals.append(fn(c))
            except Exception:  # noqa: BLE001
                vals.append(float("nan"))
        print(f"{nice:30s} " + " ".join(f"{v:>30.3f}" for v in vals))
    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        for name, c in logs:
            axs[0].plot(c["t"], c["pos_err_mm"], label=name)
            axs[1].plot(c["t"], c["ang_err_deg"], label=name)
            axs[2].plot(c["t"], c["pos_x"], label=name + " x")
        axs[0].set_ylabel("position error [mm]"), axs[1].set_ylabel("orientation error [deg]")
        axs[2].set_ylabel("x [m]"), axs[2].set_xlabel("t [s]")
        for ax in axs:
            ax.grid(True)
        axs[0].legend(fontsize=7)
        fig.savefig(png, dpi=130, bbox_inches="tight")
        print("wrote", png)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--tag", default=None, help="only rows whose tag starts with this (trajectory logs)")
    ap.add_argument("--png", default=None, help="write a comparison plot")
    a = ap.parse_args()
    if all(f.endswith("_steps.csv") for f in a.files):
        compare_steps(a.files, a.png)
    elif not any(f.endswith("_steps.csv") for f in a.files):
        compare_traj(a.files, a.tag, a.png)
    else:
        raise SystemExit("do not mix *_steps.csv files with trajectory logs")


if __name__ == "__main__":
    main()
