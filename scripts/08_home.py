#!/usr/bin/env python3
"""08_home.py  --  move the arm to the "ready" joint configuration (READY_Q in lab_config.py).

Switches to the joint_trajectory_controller and sends one joint trajectory to READY_Q
(same thing crisp_py's robot.home() does, minus its controller-list lookup).  Run it before the Cartesian tests on the REAL arm, because the controller's
nullspace pulls the elbow towards this configuration and the Cartesian scripts assume it.

The duration is chosen from the largest joint distance so that no joint moves faster than
--max-joint-speed (default 0.2 rad/s), with a minimum of 5 s (crisp_py's default).  Joint
motion is NOT checked against obstacles: clear the workspace, user stop in hand.

  ./fr3 run 08_home.py
  ./fr3 run real 08_home.py --max-joint-speed 0.15
  ./fr3 run real 08_home.py --dry-run          # only print the plan (still needs the stack)

After it finishes the joint_trajectory_controller is active and the Cartesian target (target_pose) is the home TCP pose,
repeated for one second, so switching to the Cartesian controller afterwards (by hand or by the next script) does not jump to an older target.
To set any other target without moving: 15_set_target.py.
"""
import argparse

import numpy as np

import lab_common as L
import lab_config as C


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-joint-speed", type=float, default=0.2, help="rad/s, largest joint speed on the way")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, do not move")
    ap.add_argument("--no-settle", action="store_true", help="do not publish the home pose as the Cartesian target after the move (experiment: lab-day B1)")
    L.add_common_args(ap, controller_switch=False)
    a = ap.parse_args()

    # controller=None: robot.home() switches to the joint_trajectory_controller itself
    with L.session(a, "08_home.py", controller=None) as (r, lg):
        q = np.asarray(r.joint_values, dtype=float)
        dq = np.abs(C.READY_Q - q)
        T = max(5.0, float(dq.max()) / a.max_joint_speed)
        print("current joints [rad]:", np.round(q, 3))
        print("ready   joints [rad]:", np.round(C.READY_Q, 3))
        print(f"largest joint move {dq.max():.2f} rad ({np.degrees(dq.max()):.0f} deg) -> {T:.1f} s")
        if a.dry_run:
            print("dry run: not moving.")
            return
        # Not r.home(): it reads the controller list, which is garbled across ROS distros (see 00_status.py).
        L.ensure_controller(r, "joint_trajectory_controller")
        r.joint_trajectory_controller_client.send_joint_config(
            list(C.JOINT_NAMES), [float(x) for x in C.READY_Q], T, blocking=True)
        L.hold(r)
        pose = r.end_effector_pose
        if a.no_settle:
            print("target not published (--no-settle)")
        else:
            L.settle_target(r, pose)      # repeat the target so the Cartesian controller latches the HOME pose, not an older one
            print(f"Cartesian target set to the home TCP pose: {np.round(pose.position, 4)}")
        q2 = np.asarray(r.joint_values, dtype=float)
        print("final joints [rad]:", np.round(q2, 3))
        print(f"max joint error: {np.degrees(np.abs(q2 - C.READY_Q).max()):.2f} deg")


if __name__ == "__main__":
    main()
