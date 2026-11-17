#!/usr/bin/env python3
"""Change the VR teleop scales while ./fr3 vr is running (use: ./fr3 scale [mode] --pos 0.5 --rot 0.3).

  ./fr3 scale --pos 0.5              # position scale (hand motion -> arm motion)
  ./fr3 scale --rot 0.3              # rotation scale (0 = orientation fixed)
  ./fr3 scale --pos 0.5 --rot 0.3    # both
  ./fr3 scale                        # print the current values

Sets the `pos_scale` / `rot_scale` parameters of /vr_teleop. Takes effect on the next
update; no need to release the grip.
"""
import argparse
import subprocess
import sys

MAX_POS = 1.0  # refuse anything larger; raise here if you really want more
MAX_ROT = 1.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pos", type=float, help=f"position scale, 0 < x <= {MAX_POS}")
    ap.add_argument("--rot", type=float, help=f"rotation scale, 0 <= x <= {MAX_ROT}")
    a = ap.parse_args()

    if a.pos is not None and not 0.0 < a.pos <= MAX_POS:
        ap.error(f"--pos must be in (0, {MAX_POS}] (edit MAX_POS in this file to allow more)")
    if a.rot is not None and not 0.0 <= a.rot <= MAX_ROT:
        ap.error(f"--rot must be in [0, {MAX_ROT}] (edit MAX_ROT in this file to allow more)")

    cmds = []
    if a.pos is not None:
        cmds.append(f"ros2 param set /vr_teleop pos_scale {a.pos}")
    if a.rot is not None:
        cmds.append(f"ros2 param set /vr_teleop rot_scale {a.rot}")
    if not cmds:
        cmds = ["ros2 param get /vr_teleop pos_scale", "ros2 param get /vr_teleop rot_scale"]

    # runs inside the container (./fr3 scale), where ROS and the domain of the mode are already set
    return subprocess.call(["bash", "-c", "; ".join(cmds)])


if __name__ == "__main__":
    sys.exit(main())
