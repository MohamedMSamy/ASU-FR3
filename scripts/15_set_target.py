#!/usr/bin/env python3
"""15_set_target.py  --  set the Cartesian target pose by hand: the current pose, or an x y z you type.

Why: the Cartesian controller remembers the LAST target it received, also while it is switched off, and jumps to it
when it is switched on (clipped to 0.1 m / 0.5 rad of error).  After RViz drags, joint moves or Ctrl+C that target can
be somewhere old.  This script publishes the target you want, for one second (the controller keeps the last message).

  ./fr3 run 15_set_target.py                          # target = where the TCP is right now (no jump when you switch)
  ./fr3 run 15_set_target.py --xyz 0.45 0 0.35        # ramp the target to this position (base frame, metres), orientation kept
  ./fr3 run 15_set_target.py --xyz 0.45 0 0.35 --down --yaw 20    # hand straight down, turned 20 deg about vertical
  ./fr3 run 15_set_target.py --xyz 0.45 0 0.35 --lin-speed 0.02   # slower (m/s); --ang-speed rad/s
  ./fr3 run 15_set_target.py --xyz 0.45 0 0.35 --duration 10      # take exactly 10 s (speeds follow from it)
  ./fr3 run 15_set_target.py --xyz 0.45 0 0.35 --activate         # also switch to the Cartesian controller first
  ./fr3 run 15_set_target.py --xyz 0.45 0 0.35 --instant          # NO ramp: one jump of the target (the arm may snap)
  ./fr3 run real 15_set_target.py --xyz 0.45 0 0.35 --activate --lin-speed 0.03

The target MOVES to the goal smoothly (default speeds from lab_config.py, --lin-speed / --ang-speed or --duration to change them;
the cap max_lin_speed and the safety box still apply). The ramp starts at the current TCP pose. If the Cartesian controller is
active the arm follows; if it is not, nothing moves now and the latched target simply ends at the goal. --activate switches to
the Cartesian controller at the CURRENT pose first (no jump), then ramps. --instant skips the ramp and warns when the goal is
more than 2 cm / 3 deg away. --dry-run prints the plan only.
Do not run it next to another script that publishes targets (09_rviz_goal.py): CRISP ignores a topic with two publishers.
"""
import argparse

import numpy as np

import lab_common as L
import lab_config as C


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"), help="target position [m], base frame (default: current TCP)")
    ap.add_argument("--down", action="store_true", help="hand straight down (with --yaw); default keeps the current orientation")
    ap.add_argument("--yaw", type=float, default=0.0, help="deg about vertical, only with --down")
    ap.add_argument("--instant", action="store_true", help="no ramp: publish the goal at once (the arm snaps there if the Cartesian controller is active)")
    ap.add_argument("--duration", type=float, default=None, help="seconds for the move (overrides --lin-speed/--ang-speed)")
    ap.add_argument("--activate", action="store_true", help="switch to the Cartesian controller and ramp to the target")
    ap.add_argument("--secs", type=float, default=1.0, help="how long to repeat the target message")
    ap.add_argument("--dry-run", action="store_true", help="print only, publish nothing")
    L.add_common_args(ap, controller_switch=False)
    a = ap.parse_args()
    if a.yaw and not a.down:
        ap.error("--yaw needs --down")

    L.confirm_real(a)
    r = L.connect()
    try:
        cur = r.end_effector_pose
        pos = np.asarray(a.xyz, dtype=float) if a.xyz else np.asarray(cur.position, dtype=float)
        rot = L.make_pose(pos, a.yaw).orientation if a.down else cur.orientation
        goal = L.Pose(pos, rot)
        L.check_bounds(pos)
        dp = float(np.linalg.norm(pos - cur.position)) * 100
        dr = L.orientation_error_deg(cur.orientation, rot)
        print(f"TCP now   [m]: {np.round(cur.position, 4)}   tilt from down {L.down_angle_deg(cur.orientation):.1f} deg")
        print(f"target    [m]: {np.round(pos, 4)}   tilt from down {L.down_angle_deg(rot):.1f} deg")
        print(f"distance: {dp:.1f} cm, rotation {dr:.1f} deg")
        if a.dry_run:
            print("dry run: nothing published.")
            return
        if a.activate:
            L.ensure_controller(r, C.CARTESIAN_CONTROLLER)        # holds the CURRENT pose first, so no jump
        if a.instant:
            L.settle_target(r, goal, a.secs)
            print("[lab] target published at once. " + (
                f"WARNING: it is {dp:.1f} cm / {dr:.1f} deg from the TCP: an active Cartesian controller snaps there (clipped at 10 cm)."
                if dp > 2 or dr > 3 else "It matches the TCP."))
            return
        lin, ang = a.lin_speed, a.ang_speed
        if a.duration:
            lin, ang = max(dp / 100 / a.duration, 1e-4), max(np.radians(dr) / a.duration, 1e-3)
        L.settle_target(r, cur, 0.3)                              # the ramp starts from where the TCP is, not from an old target
        T = L.ramp_to(r, goal, lin, ang)
        L.settle_target(r, goal, a.secs)
        print(f"[lab] target moved to the goal in {T:.1f} s ({lin * 100:.1f} cm/s) and held." + (
            "" if a.activate else " (nothing moves unless the Cartesian controller is active; it will hold this target.)"))
    finally:
        r.shutdown()


if __name__ == "__main__":
    main()
