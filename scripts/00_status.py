#!/usr/bin/env python3
"""00_status.py  --  read-only health check.  Run this FIRST, on the sim and on the real robot.

It does not command anything (it does not even create a crisp_py Robot, which would start
publishing hold targets).  It prints:
  * which controllers exist and which are active
  * TCP pose and how far the hand is from straight down
  * joint positions, joint-state rate, and joint efforts
  * the Cartesian controller's key parameters (gains, OSC, friction, ...)

  ./fr3 run 00_status.py
  ./fr3 run real 00_status.py
"""
import os
import sys
import time

import numpy as np

import lab_config as C

if os.environ.get("CRISP_LAB_FAKE", "0") == "1":
    print("FAKE mode: nothing to check (no ROS).")
    sys.exit(0)

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from lab_common import down_angle_deg


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("lab_status")
    state = {"pose": None, "js": None, "n_js": 0}

    def on_pose(m):
        state["pose"] = m

    def on_js(m):
        state["js"] = m
        state["n_js"] += 1

    node.create_subscription(PoseStamped, "current_pose", on_pose, qos_profile_sensor_data)
    node.create_subscription(JointState, C.JOINT_STATE_TOPIC, on_js, qos_profile_sensor_data)

    print(f"mode={C.MODE}  ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}")

    # controllers
    cli = node.create_client(ListControllers, "controller_manager/list_controllers")
    if not cli.wait_for_service(timeout_sec=5.0):
        print("!! controller_manager not found. Is the launch running? Same ROS_DOMAIN_ID? "
              "(sim=100, real=101 by default)")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("\nCONTROLLERS (ros2 control list_controllers)")
    import re
    import subprocess
    try:
        out = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True, timeout=30).stdout
    except Exception as exc:  # noqa: BLE001
        out = ""
        print(f"  could not run ros2 control: {exc}")
    motion_active = []
    for line in re.sub(r"\x1b\[[0-9;]*m", "", out).splitlines():
        parts = line.split()
        if len(parts) >= 3:
            print(f"  {parts[0]:34s} {parts[-1]:10s} {parts[1]}")
            if parts[-1] == "active" and not parts[0].endswith("broadcaster"):
                motion_active.append(parts[0])
    if len(motion_active) == 0:
        print("  -> no motion controller is active (arm is not being commanded)")
    elif len(motion_active) > 1:
        print(f"  -> WARNING: several motion controllers active: {motion_active}")
    else:
        print(f"  -> active motion controller: {motion_active[0]}")

    # topics (spin 2 s)
    t_end = time.time() + 2.0
    n0 = state["n_js"]
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    rate = (state["n_js"] - n0) / 2.0

    print("\nSTATE")
    if state["pose"] is None:
        print("  no /current_pose (pose_broadcaster not active?)")
    else:
        p = state["pose"].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        rot = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        print(f"  TCP position  [m]: {np.round(pos, 4)}")
        print(f"  angle from down  : {down_angle_deg(rot):.2f} deg")
    if state["js"] is None:
        print(f"  no {C.JOINT_STATE_TOPIC}")
    else:
        js = state["js"]
        q = np.array([js.position[js.name.index(n)] for n in C.JOINT_NAMES])
        print(f"  joints [rad]     : {np.round(q, 3)}")
        print(f"  joints [deg]     : {np.round(np.degrees(q), 1)}")
        if len(js.effort) == len(js.name):
            tau = np.array([js.effort[js.name.index(n)] for n in C.JOINT_NAMES])
            print(f"  joint effort [Nm]: {np.round(tau, 3)}")
        print(f"  joint-state rate : {rate:.0f} Hz (controller_manager runs at 1000 Hz)")

    # parameters of the Cartesian controller
    print(f"\nPARAMETERS of {C.CARTESIAN_CONTROLLER}")
    pc = node.create_client(GetParameters, f"{C.CARTESIAN_CONTROLLER}/get_parameters")
    if pc.wait_for_service(timeout_sec=3.0):
        req = GetParameters.Request()
        req.names = C.SNAPSHOT_PARAMS
        f2 = pc.call_async(req)
        rclpy.spin_until_future_complete(node, f2, timeout_sec=5.0)
        for name, val in zip(C.SNAPSHOT_PARAMS, f2.result().values):
            print(f"  {name:38s} {parameter_value_to_python(val)}")
    else:
        print("  parameter service not available")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
