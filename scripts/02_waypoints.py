#!/usr/bin/env python3
"""02_waypoints.py  --  visit your own Cartesian waypoints, hand pointing down.

Waypoints come from a CSV file (default: examples/waypoints_example.csv; a relative name is looked up in ./data first, then in examples/), one per line:

    # x, y, z, [yaw_deg], [settle_s]        (metres, base frame; '#' starts a comment)
    0.40, 0.00, 0.45
    0.45, 0.20, 0.45, 30, 1.5

  yaw_deg   rotation of the hand about the vertical axis (default 0 = the DOWN pose)
  settle_s  how long to wait at that point before measuring (default: SETTLE_S in lab_config.py)

All waypoints are checked against the safety box BEFORE anything moves.

  ./fr3 run 02_waypoints.py --file waypoints_example.csv --log wp
  ./fr3 run real 02_waypoints.py --file my_points.csv --lin-speed 0.03 --log wp
  ./fr3 run 02_waypoints.py --dry-run          # check the file, no robot, no ROS

  --repeat N   run the list N times      --no-return   do not go back to the first point
"""
import argparse
import csv
import time
from pathlib import Path

import numpy as np

import lab_common as L
import lab_config as C


def read_waypoints(path: Path):
    wps = []
    with open(path, newline="") as f:
        for lineno, line in enumerate(f, 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = [p for p in next(csv.reader([line])) if p.strip() != ""]
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                continue  # header line such as "x,y,z"
            if len(vals) < 3 or len(vals) > 5:
                raise ValueError(f"{path}:{lineno}: need 3 to 5 numbers (x,y,z[,yaw_deg][,settle_s]), got {len(vals)}")
            x, y, z = vals[:3]
            yaw = vals[3] if len(vals) >= 4 else 0.0
            settle = vals[4] if len(vals) >= 5 else C.SETTLE_S
            wps.append((np.array([x, y, z]), yaw, settle))
    if not wps:
        raise ValueError(f"{path}: no waypoints found")
    return wps


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default="waypoints_example.csv")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--no-return", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="only validate the file; no ROS")
    L.add_common_args(ap)
    a = ap.parse_args()

    path = C.find_data_file(a.file)       # lets you write --file my_points.csv: ./data/my_points.csv, then examples/
    wps = read_waypoints(path)
    for i, (p, yaw, _) in enumerate(wps, 1):
        L.check_bounds(p)
    path_len = sum(np.linalg.norm(wps[i + 1][0] - wps[i][0]) for i in range(len(wps) - 1))
    print(f"{len(wps)} waypoints from {a.file}; path length {path_len:.2f} m; "
          f"about {path_len / a.lin_speed + len(wps) * C.SETTLE_S:.0f}s per pass at {a.lin_speed} m/s")
    for i, (p, yaw, s) in enumerate(wps, 1):
        print(f"  {i:2d}: {np.round(p, 3)}  yaw {yaw:5.1f} deg  settle {s:.1f}s")
    if a.dry_run:
        print("dry run OK: all points inside the safety box.")
        return

    order = list(wps)
    if not a.no_return and len(wps) > 1:
        order.append(wps[0])

    with L.session(a, "02_waypoints.py") as (r, lg):
        print("error per axis = target - current, in the base frame")
        print(f"\n{'#':>3} {'target [m]':>24} {'pos err[mm]':>12} {'orient err[deg]':>16}   {'x[mm]':>7} {'y[mm]':>7} {'z[mm]':>7}   {'rx[deg]':>8} {'ry[deg]':>8} {'rz[deg]':>8}")
        for rep in range(a.repeat):
            for i, (p, yaw, settle) in enumerate(order, 1):
                goal = L.make_pose(p, yaw)
                L.ramp_to(r, goal, a.lin_speed, a.ang_speed, lg, label=f"move_{rep}_{i}")
                if lg:
                    lg.mark(f"settle_{rep}_{i}")
                time.sleep(settle)
                cur = r.end_effector_pose
                err = float(np.linalg.norm(cur.position - goal.position) * 1000)
                ang = L.orientation_error_deg(cur.orientation, goal.orientation)
                dp, dr = L.error_axes(cur.position, cur.orientation, goal.position, goal.orientation)
                print(f"{i:3d} {str(np.round(p, 3)):>24} {err:12.2f} {ang:16.2f}   {dp[0]:7.2f} {dp[1]:7.2f} {dp[2]:7.2f}   {dr[0]:8.3f} {dr[1]:8.3f} {dr[2]:8.3f}")
        print("final joints [rad]:", np.round(r.joint_values, 3))


if __name__ == "__main__":
    main()
