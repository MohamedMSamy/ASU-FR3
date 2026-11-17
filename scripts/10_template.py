#!/usr/bin/env python3
"""10_template.py  --  START HERE for your own experiments.

Copy this file (cp 10_template.py 11_my_test.py) and edit `my_trajectory()` or the body of
main().  Everything the other scripts use is available through `lab_common as L`:

  L.session(...)          connects, asks for confirmation on the real robot, switches to the
                          Cartesian controller, starts the CSV logger, and ALWAYS leaves the arm
                          holding its pose when the script ends or you press Ctrl+C
  L.make_pose(pos, yaw)   hand-down pose at pos (base frame, metres)
  L.ramp_to(r, pose, ...) smooth move of the TARGET to a pose (safety box enforced)
  L.stream(r, fn, T)      call fn(t) at 50 Hz for T seconds; fn returns a Pose -> sent as target
  r.set_target(pose=...)  raw crisp_py call: send one target pose (50 Hz republished by crisp_py)
  r.end_effector_pose     measured pose (.position, .orientation as scipy Rotation)
  r.joint_values          measured joints [rad];   r.set_target_joint(q) sets the nullspace/joint target
  lg.mark("text")         put a label in the log's `tag` column (use it to find phases later)

  ./fr3 run fake 10_template.py          # rehearse with no robot
  ./fr3 run 10_template.py --log tmpl
"""
import argparse
import time

import numpy as np

import lab_common as L
import lab_config as C


def my_trajectory(t: float):
    """Return the desired TCP position at time t [s].  Example: a slow vertical sine in z."""
    centre = np.array(C.READY_POS)
    return centre + np.array([0.0, 0.0, 0.03 * np.sin(2 * np.pi * 0.1 * t)])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=10.0)
    L.add_common_args(ap)
    a = ap.parse_args()

    start = L.make_pose(my_trajectory(0.0))
    with L.session(a, "10_template.py") as (r, lg):
        L.ramp_to(r, start, a.lin_speed, a.ang_speed, lg, label="to_start")
        time.sleep(1.0)
        if lg:
            lg.mark("trace")
        L.stream(r, lambda t: L.make_pose(my_trajectory(t)), a.duration)
        time.sleep(1.0)
        cur = r.end_effector_pose
        goal = my_trajectory(a.duration)
        print(f"final position error: {np.linalg.norm(cur.position - goal) * 1000:.2f} mm")


if __name__ == "__main__":
    main()
