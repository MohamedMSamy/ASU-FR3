#!/usr/bin/env python3
"""12_sweep_figure.py  --  sim vs real over a stiffness sweep: presentation figures and a table (offline, no ROS).

Input: pairs of hold logs, one pair per stiffness step, named PREFIX_k<kpos>_<krot>_{sim,real}_<time>.csv (taken with
01_hold_test.py --log PREFIX_k<kpos>_<krot>, the same pose and duration on the sim and on the real arm). The newest log of
each name is used. The gains are read from each log's .json (task.k_pos_x, task.k_rot_x, variable_max_stiffness.*).

Writes into --out (default data/logs/figures):
  sweep_error.png       settled position error vs stiffness, sim and real
  sweep_force.png       the same errors times the effective stiffness (= the force the controller thinks it needs):
                        flat = a constant disturbance force (friction), falling = just needs more stiffness
  sweep_timeseries.png  position error over time, one panel per stiffness step, sim and real
  sweep_table.csv       the numbers behind the figures (also printed)

"Settled" = the mean over the last half of the rows tagged `hold`. Effective stiffness = min(k_pos, variable_max_stiffness.translational):
the controller clamps stiffness, so k = 1200 acts like 900.

  ./fr3 run fake 12_sweep_figure.py kswp_p
  ./fr3 run fake 12_sweep_figure.py kswp_p --out /home/ros/data/figures
"""
import argparse
import csv
import glob
import json
import os
import re

import numpy as np
from scipy.spatial.transform import Rotation

LOGS = os.environ.get("FR3_LOG_DIR", "/home/ros/data/logs")
COL = {"sim": "#1f6fb2", "real": "#d9822b"}


def newest(prefix, kp, kr, mode):
    files = [f for f in glob.glob(os.path.join(LOGS, f"{prefix}_k{kp}_{kr}_{mode}_????????_??????.csv"))]
    return sorted(files)[-1] if files else None


def load(path):
    rows = list(csv.DictReader(open(path)))
    hold = [r for r in rows if r["tag"] == "hold"]
    meta = json.load(open(path[:-4] + ".json"))["controller_params"]
    return hold, meta


def stats(hold):
    """Settled (last half) mean of the position error norm [mm], the rotation error [deg] and the per-axis mean error."""
    h = hold[len(hold) // 2:]
    col = lambda names: np.array([[float(r[n]) for n in names] for r in h])
    cp, tp = col([f"pos_{a}" for a in "xyz"]), col([f"tgt_pos_{a}" for a in "xyz"])
    cq, tq = col([f"quat_{a}" for a in "xyzw"]), col([f"tgt_quat_{a}" for a in "xyzw"])
    dp = (tp - cp) * 1000.0
    dr = np.degrees((Rotation.from_quat(tq) * Rotation.from_quat(cq).inv()).as_rotvec())
    return float(np.linalg.norm(dp, axis=1).mean()), float(np.linalg.norm(dr, axis=1).mean()), dp.mean(axis=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("prefix", help="log name prefix, e.g. kswp_p")
    ap.add_argument("--out", default=os.path.join(LOGS, "figures"))
    ap.add_argument("--ts-skip-k", type=float, nargs="*", default=[300.0],
                    help="k_pos values left out of the time-series figure (default 300: its first hold starts with a settling swing)")
    a = ap.parse_args()

    steps = sorted({(m.group(1), m.group(2)) for f in glob.glob(os.path.join(LOGS, f"{a.prefix}_k*_????????_??????.csv"))
                    for m in [re.search(rf"{re.escape(a.prefix)}_k([0-9.]+)_([0-9.]+)_(?:sim|real)_", os.path.basename(f))] if m},
                   key=lambda t: (float(t[0]), float(t[1])))
    table = []
    for kp, kr in steps:
        f_sim, f_real = newest(a.prefix, kp, kr, "sim"), newest(a.prefix, kp, kr, "real")
        if not (f_sim and f_real):
            print(f"skip k{kp}_{kr}: need both a sim and a real log")
            continue
        row = {"k_pos": float(kp), "k_rot": float(kr)}
        for mode, f in (("sim", f_sim), ("real", f_real)):
            hold, meta = load(f)
            k_eff = min(float(meta["task.k_pos_x"]), float(meta.get("variable_max_stiffness.translational", 1e9)))
            err, ang, dp = stats(hold)
            row.update({f"{mode}_k_eff": k_eff, f"{mode}_pos_mm": err, f"{mode}_ang_deg": ang,
                        f"{mode}_force_N": k_eff * err / 1000.0, f"{mode}_dx": dp[0], f"{mode}_dy": dp[1], f"{mode}_dz": dp[2],
                        f"{mode}_file": os.path.basename(f)})
        table.append(row)
    if not table:
        raise SystemExit(f"no sim/real pairs for prefix '{a.prefix}' in {LOGS}")

    os.makedirs(a.out, exist_ok=True)
    print(f"{'k_pos':>6} {'k_rot':>6} | {'pos err [mm]':>19} | {'rot err [deg]':>19} | {'K_eff*err [N]':>19} | real mean error x / y / z [mm]")
    print(f"{'':>6} {'':>6} | {'sim':>9} {'real':>9} | {'sim':>9} {'real':>9} | {'sim':>9} {'real':>9} |")
    for r in table:
        print(f"{r['k_pos']:6.0f} {r['k_rot']:6.0f} | {r['sim_pos_mm']:9.2f} {r['real_pos_mm']:9.2f} | {r['sim_ang_deg']:9.2f} {r['real_ang_deg']:9.2f} | "
              f"{r['sim_force_N']:9.2f} {r['real_force_N']:9.2f} | {r['real_dx']:+7.2f} {r['real_dy']:+7.2f} {r['real_dz']:+7.2f}")
    with open(os.path.join(a.out, "sweep_table.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader(), w.writerows(table)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 14, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.25})
    labels = [f"{r['k_pos']:.0f}/{r['k_rot']:.0f}" for r in table]
    x = np.arange(len(table))

    # 1. error vs stiffness (categorical x: the sweep changes k_rot too, so label with both)
    fig, ax = plt.subplots(figsize=(9, 5.2))
    for mode in ("real", "sim"):
        ax.plot(x, [r[f"{mode}_pos_mm"] for r in table], "o-", lw=2.5, ms=8, color=COL[mode], label=f"{mode} (FR3)" if mode == "real" else "sim (MuJoCo)")
    ax.set_yscale("log"), ax.set_xticks(x), ax.set_xticklabels(labels)
    ax.set_xlabel("stiffness  k_pos [N/m] / k_rot [N m/rad]   (translational clamp 900)"), ax.set_ylabel("settled position error [mm]")
    ax.set_title("Hold error at the same pose: sim vs real"), ax.legend(frameon=False)
    fig.savefig(os.path.join(a.out, "sweep_error.png"), dpi=160, bbox_inches="tight"), plt.close(fig)

    # 2. force equivalent
    fig, ax = plt.subplots(figsize=(9, 5.2))
    w = 0.38
    ax.bar(x - w / 2, [r["sim_force_N"] for r in table], w, color=COL["sim"], label="sim (MuJoCo)")
    ax.bar(x + w / 2, [r["real_force_N"] for r in table], w, color=COL["real"], label="real (FR3)")
    ax.set_xticks(x), ax.set_xticklabels(labels)
    ax.set_xlabel("stiffness  k_pos / k_rot"), ax.set_ylabel("effective K x error [N]")
    ax.set_title("Force the controller needs to hold the pose"), ax.legend(frameon=False)
    fig.savefig(os.path.join(a.out, "sweep_force.png"), dpi=160, bbox_inches="tight"), plt.close(fig)

    # 3. time series, one panel per stiffness step (sim and real on the same axes)
    ts = [r for r in table if r["k_pos"] not in a.ts_skip_k]
    n = len(ts)
    cols = 2 if n == 4 else min(3, n)
    rows_n = -(-n // cols)
    fig, axs = plt.subplots(rows_n, cols, figsize=(5.2 * cols, 3.6 * rows_n), sharex=True, squeeze=False)
    for i, r in enumerate(ts):
        ax = axs[i // cols][i % cols]
        for mode in ("real", "sim"):
            hold, _ = load(os.path.join(LOGS, r[f"{mode}_file"]))
            t = np.array([float(h["t"]) for h in hold])
            ax.plot(t - t[0], [float(h["pos_err_mm"]) for h in hold], lw=2, color=COL[mode], label=mode)
        ax.set_title(f"k_pos {r['k_pos']:.0f} / k_rot {r['k_rot']:.0f}", fontsize=13)
        ax.set_ylim(bottom=0)
        if i % cols == 0:
            ax.set_ylabel("position error [mm]")
        if i // cols == rows_n - 1:
            ax.set_xlabel("time in hold [s]")
        if i == 0:
            ax.legend(frameon=False)
    for j in range(n, rows_n * cols):
        axs[j // cols][j % cols].axis("off")
    fig.suptitle("Position error during the hold, sim vs real", fontsize=16)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "sweep_timeseries.png"), dpi=160, bbox_inches="tight"), plt.close(fig)
    print(f"\nwrote sweep_error.png, sweep_force.png, sweep_timeseries.png, sweep_table.csv in {a.out}")


if __name__ == "__main__":
    main()
