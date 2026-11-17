#!/usr/bin/env python3
"""08.1_home.py  --  like 08_home.py, but through the joint_impedance_controller instead of the joint_trajectory_controller.

The joint impedance controller is a spring on every joint towards /target_joint (see 05_deadband_joint.py), and it
carries the gravity term when use_gravity_compensation is on.  The joint trajectory controller is a plain PID with no gravity term.
This script ramps the joint target from where the arm is to READY_Q (smooth S-curve, no joint faster than --max-joint-speed)
and prints how far the arm ends from READY_Q.  The spring only pulls with at most max_tau (5 N m) per joint, so the target must move slowly:
the default is the same 0.2 rad/s as 08_home.py.

  ./fr3 run 08.1_home.py
  ./fr3 run 08.1_home.py --max-joint-speed 0.1
  ./fr3 run 08.1_home.py --dry-run

Afterwards the joint_impedance_controller is the active controller and the Cartesian target is the home TCP pose (as in 08_home.py).
"""
import argparse
import time

import numpy as np

import lab_common as L
import lab_config as C


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-joint-speed", type=float, default=0.2, help="rad/s, largest joint speed on the way (peak of the S-curve is 1.875x the average)")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, do not move")
    ap.add_argument("--no-settle", action="store_true", help="do not publish the home pose as the Cartesian target after the move")
    L.add_common_args(ap, controller_switch=False)
    a = ap.parse_args()

    with L.session(a, "08.1_home.py", controller=None) as (r, lg):
        q0 = np.asarray(r.joint_values, dtype=float)
        dq = C.READY_Q - q0
        # minimum-jerk S-curve s(u) = 10u^3 - 15u^4 + 6u^5: its peak speed is 1.875 * distance / T, so T is chosen for the peak.
        T = max(5.0, 1.875 * float(np.abs(dq).max()) / a.max_joint_speed)
        print("current joints [rad]:", np.round(q0, 3))
        print("ready   joints [rad]:", np.round(C.READY_Q, 3))
        print(f"largest joint move {np.abs(dq).max():.2f} rad ({np.degrees(np.abs(dq).max()):.0f} deg) -> {T:.1f} s")
        if a.dry_run:
            print("dry run: not moving.")
            return
        L.ensure_controller(r, "joint_impedance_controller")
        n = int(round(T * C.PUBLISH_HZ))
        t0 = time.perf_counter()
        for k in range(1, n + 1):
            u = k / n
            s = 10 * u**3 - 15 * u**4 + 6 * u**5
            r.set_target_joint(q0 + s * dq)
            L._sleep_until(t0 + k / C.PUBLISH_HZ)
        for _ in range(int(C.PUBLISH_HZ)):          # keep the final target for one more second so the arm settles on it
            r.set_target_joint(C.READY_Q)
            time.sleep(1.0 / C.PUBLISH_HZ)
        pose = r.end_effector_pose
        if a.no_settle:
            print("target not published (--no-settle)")
        else:
            L.settle_target(r, pose)
            print(f"Cartesian target set to the home TCP pose: {np.round(pose.position, 4)}")
        q2 = np.asarray(r.joint_values, dtype=float)
        print("final joints [rad]:", np.round(q2, 3))
        print(f"max joint error: {np.degrees(np.abs(q2 - C.READY_Q).max()):.2f} deg")


if __name__ == "__main__":
    main()
