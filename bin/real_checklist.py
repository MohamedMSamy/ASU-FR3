#!/usr/bin/env python3
"""real_checklist.py  --  what `./fr3 up real` prints before it asks you to type 'go'.

Runs in a one-off container (no ROS, no network traffic to the robot). It reads the FILES THE STACK WILL USE (your ./config
overrides the baked defaults, exactly like the launch does) and prints: the safety box and speed caps of the real arm, where
lab_config.py comes from, the controller file with its gains and the settings that matter on the real arm, the robot IP and the
ROS domain, and the checklist for the lab. It only informs: the decision is yours.

Environment: CONTROLLERS_FILE (default controllers_real.yaml), ROBOT_IP, ROS_DOMAIN_ID.
"""
import filecmp
import os
import sys

import yaml

OVERRIDE, BAKED = "/ws_override/config", "/opt/asu_fr3/config"


def locate(name):
    """(path, 'override (differs from baked)' | 'override (same as baked)' | 'baked default')"""
    o, b = os.path.join(OVERRIDE, name), os.path.join(BAKED, name)
    if os.path.isfile(o):
        if os.path.isfile(b):
            return o, "override " + ("(same as baked)" if filecmp.cmp(o, b, shallow=False) else "(DIFFERS from the baked default)")
        return o, "override (no baked file of that name)"
    return b, "baked default"


def main():
    ip = os.environ.get("ROBOT_IP", "172.16.0.2")
    dom = os.environ.get("ROS_DOMAIN_ID", "101")
    cf_name = os.environ.get("CONTROLLERS_FILE", "controllers_real.yaml")
    warn = []

    print(f"  robot IP        : {ip}")
    print(f"  ROS domain      : {dom}   (DDS: Cyclone on the loopback interface, as in the sim)")

    # ---- lab_config.py: the safety box
    lc_path, lc_src = locate("lab_config.py")
    os.environ["CRISP_LAB_MODE"] = "real"
    sys.path.insert(0, os.path.dirname(lc_path))
    import lab_config as C  # noqa: E402

    lim = C.LIMITS["real"]
    lo, hi = lim["bounds_min"], lim["bounds_max"]
    print(f"\n  lab_config.py   : {lc_path}  [{lc_src}]")
    print("  SAFETY BOX (the scripts refuse to command anything outside; the controller itself has no box):")
    print(f"      x {lo[0]:+.2f} .. {hi[0]:+.2f} m    y {lo[1]:+.2f} .. {hi[1]:+.2f} m    z {lo[2]:+.2f} .. {hi[2]:+.2f} m   (base frame)")
    print(f"      speed: default {lim['lin_speed']} m/s, cap {lim['max_lin_speed']} m/s; angular {lim['ang_speed']} rad/s; max accel {lim['max_accel']} m/s^2")
    print("      this is the ASU lab box: VERIFY IT FOR YOUR TABLE AND FIXTURES before you continue.")
    rp = C.READY_POS
    if any(rp[i] < lo[i] or rp[i] > hi[i] for i in range(3)):
        warn.append(f"READY_POS {list(map(float, rp))} is outside the real safety box: the hold/waypoint scripts would refuse it")

    # ---- the controllers file
    cf_path, cf_src = locate(cf_name)
    print(f"\n  controller file : {cf_path}  [{cf_src}]")
    try:
        root = yaml.safe_load(open(cf_path))
        root = root.get("/**", root)
        cm = root["controller_manager"]["ros__parameters"]
        cart = root["cartesian_impedance_controller"]["ros__parameters"]
        task = cart.get("task", {})
        kp = [task.get(f"k_pos_{a}", 500.0) for a in "xyz"]
        kr = [task.get(f"k_rot_{a}", 30.0) for a in "xyz"]
        overrun = cm.get("enable_overrun")
        osc = cart.get("use_operational_space", False)
        grav = cart.get("use_gravity_compensation", False)
        fric = cart.get("use_friction", False)
        vmax = cart.get("variable_max_stiffness", {})
        kmax = (vmax.get("translational", 900.0), vmax.get("rotational", 60.0))
        print(f"      cartesian_impedance_controller: k_pos {kp} N/m, k_rot {kr} N m/rad   (stiffness is clamped at {kmax[0]:g} / {kmax[1]:g})")
        print(f"      enable_overrun {overrun}   use_operational_space {osc}   use_friction {fric}   use_gravity_compensation {grav}")
        print(f"      introspection {cart.get('enable_introspection', False)} (needed by ./fr3 rec for the commanded torque and missed cycles)")
        if overrun is not False:
            warn.append("enable_overrun is not false: on the real arm libfranka already paces the loop at 1 kHz; with true two clocks beat (use controllers_real.yaml)")
        if grav:
            warn.append("use_gravity_compensation is TRUE: libfranka already compensates gravity in the real arm, so this is DOUBLE compensation (the arm is pushed up at activation). The launch will refuse it; use controllers_real.yaml")
        if osc:
            warn.append("use_operational_space is TRUE: OSC has never been run on the real arm (it was unstable in the sim)")
        if max(kp) > 300 or max(kr) > 15:
            warn.append(f"the gains in this file (k_pos {max(kp):g}, k_rot {max(kr):g}) are above the low start of the lab plan (300 / 15): "
                        "after the stack is up run  ./fr3 setk real 300 15  BEFORE the first script, and check with  ./fr3 run real 00_status.py")
    except Exception as exc:  # noqa: BLE001
        warn.append(f"could not read the gains from {cf_path}: {exc}")
    print("      until a script activates cartesian_impedance_controller, the arm is held by the joint PD (joint_trajectory_controller).")

    # ---- the checklist
    print("""
  CHECKLIST (tick every line yourself)
    [ ] Desk: FCI activated, joints unlocked (brakes open), no error shown, the Franka Hand preset set (it drives libfranka's gravity compensation)
    [ ] the user stop / external activation device is IN YOUR HAND and works
    [ ] the workspace is clear, nothing inside the safety box above
    [ ] nothing else publishes to the arm: no other stack (old crisp_lab, another terminal), no leftover script (./fr3 monitor shows the target publishers)
    [ ] after start: ./fr3 setk real 300 15, then ./fr3 run real 00_status.py (gains, rate about 1000 Hz, TCP at home z = 0.4869), ./fr3 run real 08_home.py between tests
    [ ] stop with Ctrl+C in this terminal or ./fr3 down real (never docker kill)""")
    if warn:
        print("\n  WARNINGS")
        for w in warn:
            print("    !! " + w)
    print()


if __name__ == "__main__":
    main()
