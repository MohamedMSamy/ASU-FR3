#!/usr/bin/env python3
"""01_hold_test.py  --  go to a pose, hold it, and watch the error for N seconds.

The simplest "is the controller good?" test, and the baseline to compare sim vs real.
Prints angle error (to the commanded orientation) and position error once per second,
then a summary: final error, and how fast the error is still drifting (a slow drift means
the arm is creeping out of a friction dead band).

  ./fr3 run 01_hold_test.py --duration 20 --log hold
  ./fr3 run real 01_hold_test.py --duration 20 --log hold --pos 0.40 0 0.45

Edit the defaults in lab_config.py, or pass --pos / --yaw-deg.
"""
import argparse
import time

import numpy as np

import lab_common as L
import lab_config as C
from lab_analysis import drift_rate


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pos", nargs=3, type=float, default=list(C.READY_POS), metavar=("X", "Y", "Z"))
    ap.add_argument("--yaw-deg", type=float, default=0.0, help="rotation of the hand about vertical")
    ap.add_argument("--duration", type=float, default=20.0, help="seconds to hold and watch")
    L.add_common_args(ap)
    a = ap.parse_args()

    goal = L.make_pose(a.pos, a.yaw_deg)
    L.check_bounds(goal.position)

    with L.session(a, "01_hold_test.py") as (r, lg):
        T = L.ramp_to(r, goal, a.lin_speed, a.ang_speed, lg, label="ramp")
        print(f"ramped to {np.round(goal.position, 3)} in {T:.1f}s; now holding for {a.duration:.0f}s")
        if lg:
            lg.mark("hold")
        print("error per axis = target - current, in the base frame (rotation: about base x, y, z)")
        print(f"{'t[s]':>6} {'angle err[deg]':>15} {'pos err[mm]':>12}   {'x[mm]':>7} {'y[mm]':>7} {'z[mm]':>7}   {'rx[deg]':>8} {'ry[deg]':>8} {'rz[deg]':>8}")
        t0 = time.time()
        ts, angs, errs, dps, drs = [], [], [], [], []
        while time.time() - t0 < a.duration:
            cur = r.end_effector_pose
            ang = L.orientation_error_deg(cur.orientation, goal.orientation)
            err = float(np.linalg.norm(cur.position - goal.position) * 1000)
            t = time.time() - t0
            dp, dr = L.error_axes(cur.position, cur.orientation, goal.position, goal.orientation)
            ts.append(t), angs.append(ang), errs.append(err), dps.append(dp), drs.append(dr)
            print(f"{t:6.1f} {ang:15.2f} {err:12.2f}   {dp[0]:7.2f} {dp[1]:7.2f} {dp[2]:7.2f}   {dr[0]:8.3f} {dr[1]:8.3f} {dr[2]:8.3f}")
            time.sleep(1.0)
        print("\nSUMMARY")
        print(f"  final angle error : {angs[-1]:.2f} deg   (drift {drift_rate(ts, angs):+.3f} deg/s over the last half)")
        print(f"  final position err: {errs[-1]:.2f} mm    (drift {drift_rate(ts, errs):+.3f} mm/s)")
        half = len(ts) // 2
        mp, mr = np.mean(dps[half:], axis=0), np.mean(drs[half:], axis=0)
        print(f"  mean error per axis over the last half (target - current):")
        print(f"    position [mm] : x {mp[0]:+.2f}  y {mp[1]:+.2f}  z {mp[2]:+.2f}")
        print(f"    rotation [deg]: rx {mr[0]:+.3f}  ry {mr[1]:+.3f}  rz {mr[2]:+.3f}")


if __name__ == "__main__":
    main()
