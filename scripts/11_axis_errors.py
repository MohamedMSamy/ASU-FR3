#!/usr/bin/env python3
"""11_axis_errors.py  --  per-axis error of any lab log (offline, no ROS, works on old logs too).

For every position axis (x, y, z, in mm) and every rotation axis (about base x, y, z, in deg) it prints the mean, the standard
deviation and the largest absolute error over the settled part of the run. Error = target - current, in the base frame
(rotation: the rotation vector that takes the current orientation to the target). It reads the columns every lab log has
(pos_*, quat_*, tgt_pos_*, tgt_quat_*), so it also works on the logs recorded before the per-axis columns existed.

  ./fr3 run fake 11_axis_errors.py /home/ros/data/logs/hold_k300_15_real_20261022_134528.csv
  ./fr3 run fake 11_axis_errors.py "/home/ros/data/logs/kswp_p_k300.0_15.0_real_*.csv" --tag hold   # several files, only rows tagged "hold"
  (needs no stack: `fake` starts a one-off container that only mounts ./data; plain `python3 11_axis_errors.py <files>` works on any machine with numpy and scipy)
By default the last two thirds of each file are used (the settled part); --tag NAME uses the rows with that tag instead.
--forces also prints the spring force F = K e (N) and torque M = K_rot e_rot (N m) the controller asked for, from the gains stored in the log's .json
(task.k_pos_*, task.k_rot_*, clamped at variable_max_stiffness): works on logs of the OLD stack too. Estimate: base-frame error, gains from the start of the log.
"""
import argparse
import csv
import glob

import numpy as np
from scipy.spatial.transform import Rotation


def load(path, tag=None):
    rows = list(csv.DictReader(open(path)))
    if tag:
        rows = [r for r in rows if r["tag"] == tag]
    else:
        rows = rows[len(rows) // 3:]
    if not rows:
        raise SystemExit(f"{path}: no rows" + (f" with tag '{tag}'" if tag else ""))
    col = lambda names: np.array([[float(r[n]) for n in names] for r in rows])
    cur_p, tgt_p = col([f"pos_{a}" for a in "xyz"]), col([f"tgt_pos_{a}" for a in "xyz"])
    cur_q, tgt_q = col([f"quat_{a}" for a in "xyzw"]), col([f"tgt_quat_{a}" for a in "xyzw"])
    dp = (tgt_p - cur_p) * 1000.0
    dr = np.degrees((Rotation.from_quat(tgt_q) * Rotation.from_quat(cur_q).inv()).as_rotvec())
    return dp, dr


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("logs", nargs="+", help="log CSV files (globs are expanded)")
    ap.add_argument("--tag", help="use only the rows with this tag (e.g. hold)")
    ap.add_argument("--forces", action="store_true", help="also print the spring force / torque K*e (needs the .json next to each log)")
    a = ap.parse_args()
    files = sorted({f for pat in a.logs for f in (glob.glob(pat) or [pat])})
    print("error = target - current, base frame; mean / std / max|.| over the settled part")
    print(f"{'file':44} {'':9} {'x':>18} {'y':>18} {'z':>18}")
    for f in files:
        dp, dr = load(f, a.tag)
        name = f.split("/")[-1][:44]
        for label, d, unit in (("pos [mm]", dp, ""), ("rot [deg]", dr, "")):
            cells = [f"{d[:, i].mean():+7.3f} {d[:, i].std():6.3f} {np.abs(d[:, i]).max():6.2f}" for i in range(3)]
            print(f"{name:44} {label:9} {cells[0]:>18} {cells[1]:>18} {cells[2]:>18}")
            name = ""
        print(f"{'':44} {'n':9} {len(dp)} rows")
        if a.forces:
            try:
                import json
                pr = json.load(open(f[:-4] + ".json"))["controller_params"]
                num = lambda key, default=None: default if pr.get(key) is None else float(pr[key])  # noqa: E731  (null = not recorded)
                kp = np.minimum([num(f"task.k_pos_{c}") for c in "xyz"], num("variable_max_stiffness.translational", 900.0))
                kr = np.minimum([num(f"task.k_rot_{c}") for c in "xyz"], num("variable_max_stiffness.rotational", 60.0))
                F, M = kp * dp / 1000.0, kr * np.radians(dr)
                print(f"{'':44} {'F [N]':9} " + " ".join(f"{F[:, i].mean():+8.2f} +-{F[:, i].std():5.2f}" for i in range(3))
                      + f"   |F| mean {np.linalg.norm(F, axis=1).mean():6.2f} max {np.linalg.norm(F, axis=1).max():6.2f}")
                print(f"{'':44} {'M [N m]':9} " + " ".join(f"{M[:, i].mean():+8.3f} +-{M[:, i].std():5.3f}" for i in range(3))
                      + f"   |M| mean {np.linalg.norm(M, axis=1).mean():6.3f} max {np.linalg.norm(M, axis=1).max():6.3f}")
            except Exception as exc:  # noqa: BLE001
                print(f"{'':44} forces: not available ({exc})")


if __name__ == "__main__":
    main()
