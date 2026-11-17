#!/usr/bin/env python3
"""07_record_waypoints.py  --  teach waypoints by hand-guiding the arm, save them as a CSV.

Switches to the `gravity_compensation` controller (no position stiffness, so you can push the
arm around by hand on the REAL robot; Franka itself compensates gravity).  Move the hand where
you want it, press Enter to record the pose, repeat, type q to finish.  The CSV can be played
back with 02_waypoints.py.

  ./fr3 run real 07_record_waypoints.py --out my_points.csv

SAFETY: hand-guiding on hardware needs the user stop in your hand.  The orientation is still
held softly (k_rot of the gravity_compensation block in controllers.yaml); lower it to 0 to
rotate the hand freely.  In the simulator nothing pushes the arm, so this only records the
current pose (useful to check the script).  Only x, y, z and the yaw about the vertical are
saved: the playback always keeps the hand pointing down.
"""
import argparse
from pathlib import Path

import numpy as np

import lab_common as L
import lab_config as C


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="my_points.csv", help="CSV to write (relative names go in ./data)")
    ap.add_argument("--append", action="store_true")
    L.add_common_args(ap, controller_switch=False)
    a = ap.parse_args()
    out = Path(a.out)
    if not out.is_absolute():
        out = C.DATA_DIR / out
    out.parent.mkdir(parents=True, exist_ok=True)

    with L.session(a, "07_record_waypoints.py", controller="gravity_compensation") as (r, lg):
        rows = []
        print("Hand-guide the arm.  Enter = record this pose, q = save and quit.")
        while True:
            cmd = input(f"[{len(rows)} recorded] > ").strip().lower()
            if cmd == "q":
                break
            cur = r.end_effector_pose
            tilt = L.down_angle_deg(cur.orientation)
            yaw = float(np.degrees((cur.orientation * C.DOWN.inv()).as_rotvec()[2]))
            p = cur.position
            inside = bool(np.all(p >= C.BOUNDS_MIN) and np.all(p <= C.BOUNDS_MAX))
            print(f"   pos {np.round(p, 4)}  yaw {yaw:6.1f} deg  hand {tilt:4.1f} deg from down"
                  + ("" if inside else "   !! OUTSIDE the safety box (will be refused on playback)"))
            if tilt > 5:
                print("   note: hand is tilted; playback will point it straight down")
            rows.append((p[0], p[1], p[2], yaw))
    if rows:
        mode = "a" if a.append else "w"
        with open(out, mode) as f:
            if mode == "w":
                f.write("# x, y, z, yaw_deg   recorded by 07_record_waypoints.py\n")
            for x, y, z, yaw in rows:
                f.write(f"{x:.4f}, {y:.4f}, {z:.4f}, {yaw:.1f}\n")
        print(f"saved {len(rows)} waypoints to {out}")
    else:
        print("nothing recorded.")


if __name__ == "__main__":
    main()
