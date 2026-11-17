"""lab_common.py  --  helpers shared by every script in scripts/.

You normally do not edit this.  Settings live in lab_config.py.

Key ideas
  * connect()   builds a crisp_py Robot (or a FakeRobot if CRISP_LAB_FAKE=1)
  * session()   connect + safety confirmation + controller switch + optional CSV log,
                and ALWAYS leaves the robot holding its current pose when the script ends
  * ramp_to / stream   smooth target streaming at 50 Hz, with the safety box enforced
  * Logger      100 Hz CSV of pose, target, joints, velocities, joint efforts
"""
from __future__ import annotations

import argparse
import csv
import datetime
import json
import os
import sys
import threading
import time
from contextlib import contextmanager

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import lab_config as C

FAKE = os.environ.get("CRISP_LAB_FAKE", "0") == "1"

if FAKE:
    from lab_fake import Pose  # noqa: F401
else:
    try:
        from crisp_py.utils.geometry import Pose  # noqa: F401
    except ImportError:
        # No ROS/crisp_py on this machine: fine for --dry-run checks.  connect() will fail loudly
        # if you try to talk to a robot without them.
        from lab_fake import Pose  # noqa: F401


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------
def make_pose(pos, yaw_deg: float = 0.0) -> Pose:
    """Hand pointing down, rotated `yaw_deg` about the vertical (base z) axis."""
    rot = Rotation.from_euler("z", yaw_deg, degrees=True) * C.DOWN
    return Pose(np.asarray(pos, dtype=float), rot)


def down_angle_deg(rot: Rotation) -> float:
    """Angle between the tool z axis and straight down, degrees."""
    z = rot.apply([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(-z[2], -1.0, 1.0))))


def orientation_error_deg(a: Rotation, b: Rotation) -> float:
    """Smallest rotation angle between two orientations, degrees."""
    return float(np.degrees((a.inv() * b).magnitude()))


def error_axes(cur_pos, cur_rot: Rotation, tgt_pos, tgt_rot: Rotation):
    """Error per axis, in the base frame: target minus current.

    Returns (dp, dr): dp = target - current position in mm (x, y, z); dr = the rotation that takes the current orientation
    to the target, as a rotation vector in degrees about the base x, y, z axes. Positive dp_x means the hand is still short
    of the target along +x; positive dr_z means it still has to turn counter-clockwise about +z (seen from above).
    """
    dp = (np.asarray(tgt_pos, dtype=float) - np.asarray(cur_pos, dtype=float)) * 1000.0
    dr = np.degrees((tgt_rot * cur_rot.inv()).as_rotvec())
    return dp, dr


def check_bounds(p) -> None:
    p = np.asarray(p, dtype=float)
    if np.any(p < C.BOUNDS_MIN) or np.any(p > C.BOUNDS_MAX):
        raise ValueError(
            f"point {np.round(p, 3)} is outside the safety box "
            f"{C.BOUNDS_MIN} .. {C.BOUNDS_MAX} (mode={C.MODE}). "
            "Edit LIMITS in lab_config.py if this is intended."
        )


def _smooth(u: float) -> float:
    """Minimum-jerk time scaling 0..1."""
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


# ----------------------------------------------------------------------------
# Connecting
# ----------------------------------------------------------------------------
def connect(timeout: float = 10.0):
    """Return a ready robot object."""
    if FAKE:
        from lab_fake import FakeRobot
        print("[lab] FAKE robot (CRISP_LAB_FAKE=1): no ROS, no sim.")
        return FakeRobot()
    from crisp_py.robot import Robot
    from crisp_py.robot.robot_config import FrankaConfig

    cfg = FrankaConfig(
        joint_names=C.JOINT_NAMES,
        target_frame=C.TARGET_FRAME,
        base_frame=C.BASE_FRAME,
        current_joint_topic=C.JOINT_STATE_TOPIC,
        use_tf_pose=False,
    )
    r = Robot(namespace="", robot_config=cfg)
    r.wait_until_ready(timeout=timeout)
    _attach_effort_listener(r)
    return r


def _attach_effort_listener(r) -> None:
    """crisp_py does not expose joint efforts, so listen to the joint-state topic ourselves."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    r._lab_effort = np.full(len(C.JOINT_NAMES), np.nan)

    def cb(msg):
        e = np.full(len(C.JOINT_NAMES), np.nan)
        for name, val in zip(msg.name, msg.effort):
            if name in C.JOINT_NAMES:
                e[C.JOINT_NAMES.index(name)] = val
        r._lab_effort = e

    r.node.create_subscription(JointState, C.JOINT_STATE_TOPIC, cb, qos_profile_sensor_data)


def hold(r) -> None:
    """Make the controller hold wherever the arm is right now (pose and joint targets)."""
    try:
        r.set_target(pose=r.end_effector_pose)
        r.set_target_joint(r.joint_values)
    except Exception as exc:  # noqa: BLE001
        print(f"[lab] could not set hold target: {exc}")


def settle_target(r, pose=None, secs: float = 1.0) -> None:
    """Publish `pose` (default: the current pose) as the Cartesian target repeatedly for `secs` seconds.

    hold() sends the target once; a single best-effort message right before the script exits can be missed,
    and the controller then keeps an older target (it jumps there when it is switched on).  Repeating it for a
    second makes sure the latched target is this one.
    """
    pose = pose if pose is not None else r.end_effector_pose
    n = max(1, int(round(secs * C.PUBLISH_HZ)))
    t0 = time.perf_counter()
    for k in range(1, n + 1):
        r.set_target(pose=pose)
        _sleep_until(t0 + k / C.PUBLISH_HZ)


MOTION_CONTROLLERS = ["cartesian_impedance_controller", "joint_impedance_controller",
                      "gravity_compensation", "joint_trajectory_controller",
                      "torque_feedback_controller"]


def list_controllers(r):
    """[(name, state)] from the controller manager, or None if the answer is garbled.

    In the old stack the client (pixi, humble) differed from the stack (jazzy) and the ListControllers reply changed
    between the two, giving blank names ("string data is not null-terminated").  This image runs both on jazzy, but
    the check is kept: it returns None if the answer is garbled instead of wrong data.
    """
    try:
        lst = r.controller_switcher_client.get_controller_list()
    except Exception:  # noqa: BLE001
        return None
    out = [(c.name, c.state) for c in lst]
    if not out or any((not n) or (st not in ("active", "inactive", "unconfigured", "finalized")) for n, st in out):
        return None
    return out


def verify_active(name: str) -> None:
    """Independently check that `name` is really the active motion controller.

    After the switch (sent without reading the list, see ensure_controller) this asks the controller
    manager with `ros2 control list_controllers`.  Raises if `name` is not active or another motion
    controller still is.  If the check itself cannot run (timeout) it only warns.  Skip it with
    CRISP_LAB_NO_VERIFY=1.
    """
    if FAKE or os.environ.get("CRISP_LAB_NO_VERIFY", "0") == "1":
        return
    import re
    import subprocess

    try:
        out = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True, timeout=30).stdout
    except Exception as exc:  # noqa: BLE001
        print(f"[lab] WARNING: could not verify the active controller ({exc}); run `./fr3 ctl` yourself")
        return
    states = {}
    for line in re.sub(r"\x1b\[[0-9;]*m", "", out).splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[-1] in ("active", "inactive", "unconfigured", "finalized"):
            states[parts[0]] = parts[-1]
    if not states:
        print(f"[lab] WARNING: could not read the controller list via ros2 control; verify by hand")
        return
    if states.get(name) != "active":
        raise RuntimeError(f"{name} is NOT active after the switch (state: {states.get(name)}). "
                           f"Controller states: {states}")
    others = [c for c in MOTION_CONTROLLERS if c != name and states.get(c) == "active"]
    if others:
        raise RuntimeError(f"more than one motion controller is active: {[name] + others}")
    print(f"[lab] verified via ros2 control: {name} is the only active motion controller")


def ensure_controller(r, name: str) -> None:
    """Make `name` the active motion controller (other motion controllers get deactivated).

    Targets are synced to the current state FIRST, so the controller never sees a stale
    target at the instant it activates.  If the controller list can be read (same ROS
    distro on both sides) crisp_py's normal switch is used.  If it is garbled (humble client
    vs jazzy stack, see list_controllers) we send the switch request without looking at the
    list: deactivate every other known motion controller, activate `name`, BEST_EFFORT.
    """
    hold(r)
    time.sleep(0.3)
    # The old (humble) client could not decode this stack's ListControllers reply and printed rcutils errors, so:
    # switch without reading the list, then verify with `ros2 control list_controllers`.
    # Set CRISP_LAB_READ_LIST=1 to use crisp_py's normal switch when client and stack versions match (untested here).
    if os.environ.get("CRISP_LAB_READ_LIST", "0") == "1" and list_controllers(r) is not None:
        r.controller_switcher_client.switch_controller(name)
    else:
        others = [c for c in MOTION_CONTROLLERS if c != name]
        ok = r.controller_switcher_client._switch_controller(others, [name])
        if not ok:
            raise RuntimeError(f"switch to {name} failed; check `./fr3 ctl` and `./fr3 logs`")
        time.sleep(0.5)
        verify_active(name)
    time.sleep(0.5)
    hold(r)
    print(f"[lab] active motion controller: {name}")


def controller_params(r, which: str = "cartesian") -> dict:
    """Read the controller's ROS parameters (best effort)."""
    client = (r.cartesian_controller_parameters_client if which == "cartesian"
              else r.joint_controller_parameters_client)
    try:
        values = client.get_parameters(C.SNAPSHOT_PARAMS)
        return dict(zip(C.SNAPSHOT_PARAMS, values))
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}


# ----------------------------------------------------------------------------
# Streaming targets
# ----------------------------------------------------------------------------
def _sleep_until(t_abs: float) -> None:
    dt = t_abs - time.perf_counter()
    if dt > 0:
        time.sleep(dt)


def stream(r, pose_fn, duration: float, hz: float = C.PUBLISH_HZ) -> None:
    """Call pose_fn(t) at `hz` for `duration` seconds and send each pose as the target."""
    n = max(1, int(round(duration * hz)))
    t0 = time.perf_counter()
    for k in range(1, n + 1):
        pose = pose_fn(k / hz)
        check_bounds(pose.position)
        r.set_target(pose=pose)
        _sleep_until(t0 + k / hz)


def ramp_to(r, goal: Pose, lin_speed: float | None = None, ang_speed: float | None = None,
            logger=None, label: str = "") -> float:
    """Move the TARGET smoothly from its current value to `goal`.  Returns the duration."""
    lin_speed = lin_speed or C.LIM["lin_speed"]
    ang_speed = ang_speed or C.LIM["ang_speed"]
    if lin_speed > C.LIM["max_lin_speed"]:
        raise ValueError(f"lin_speed {lin_speed} m/s above cap {C.LIM['max_lin_speed']} (lab_config.py)")
    check_bounds(goal.position)
    start = r.target_pose
    dist = float(np.linalg.norm(goal.position - start.position))
    ang = float((start.orientation.inv() * goal.orientation).magnitude())
    T = max(dist / lin_speed, ang / ang_speed, 0.3)
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([start.orientation, goal.orientation]))
    if logger:
        logger.mark(label or "ramp")

    def fn(t):
        s = _smooth(min(t / T, 1.0))
        return Pose((1 - s) * start.position + s * goal.position, slerp([s])[0])

    stream(r, fn, T)
    r.set_target(pose=goal)
    return T


# ----------------------------------------------------------------------------
# Logging
# ----------------------------------------------------------------------------
class Logger:
    """Samples the robot at C.LOG_HZ in a background thread and writes a CSV on stop()."""

    COLS = (["t", "tag"]
            + [f"pos_{a}" for a in "xyz"] + [f"quat_{a}" for a in "xyzw"]
            + [f"tgt_pos_{a}" for a in "xyz"] + [f"tgt_quat_{a}" for a in "xyzw"]
            + ["pos_err_mm", "ang_err_deg", "down_angle_deg"]
            + ["err_x_mm", "err_y_mm", "err_z_mm", "err_rx_deg", "err_ry_deg", "err_rz_deg"]
            + ["f_x_N", "f_y_N", "f_z_N", "m_x_Nm", "m_y_Nm", "m_z_Nm"]
            + [f"q{i}" for i in range(1, 8)] + [f"dq{i}" for i in range(1, 8)]
            + [f"tau{i}" for i in range(1, 8)])

    def __init__(self, r, name: str, script: str = "", extra_meta: dict | None = None):
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        C.LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.base = C.LOG_DIR / f"{name}_{C.MODE}_{stamp}"
        self.r = r
        self.rows: list[list] = []
        self.tag = ""
        self._stop = threading.Event()
        self._t0 = time.perf_counter()
        self.meta = {
            "name": name, "script": script, "mode": C.MODE, "fake": FAKE,
            "started": stamp, "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "argv": sys.argv, "controller_params": controller_params(r),
        }
        if extra_meta:
            self.meta.update(extra_meta)
        self._k = self._spring_gains(self.meta["controller_params"])
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @staticmethod
    def _spring_gains(params: dict):
        """(k_pos[3], k_rot[3]) the controller really uses (the stiffness is clamped at variable_max_stiffness), or None if unknown."""
        try:
            num = lambda key, default=None: default if params.get(key) is None else float(params[key])  # noqa: E731  (None = not readable)
            kp = [num(f"task.k_pos_{a}") for a in "xyz"]
            kr = [num(f"task.k_rot_{a}") for a in "xyz"]
            if None in kp or None in kr:
                return None
            return np.minimum(kp, num("variable_max_stiffness.translational", 900.0)), np.minimum(kr, num("variable_max_stiffness.rotational", 60.0))
        except Exception:  # noqa: BLE001
            return None

    def mark(self, tag: str) -> None:
        self.tag = tag

    def _sample(self) -> list | None:
        r = self.r
        try:
            cur = r.end_effector_pose
            tgt = r.target_pose
            q, dq = r.joint_values, r.joint_velocities
        except Exception:  # noqa: BLE001
            return None
        tau = getattr(r, "_lab_effort", np.full(7, np.nan))
        pos_err = float(np.linalg.norm(cur.position - tgt.position) * 1000.0)
        ang_err = orientation_error_deg(cur.orientation, tgt.orientation)
        dp, dr = error_axes(cur.position, cur.orientation, tgt.position, tgt.orientation)
        # spring force/torque the controller asks for, F = K e and M = K_rot e_rot (an estimate: gains read at the start of the log, error in
        # the base frame while the controller works in the tool frame and clips the error at 0.1 m / 0.5 rad)
        if self._k is None:
            fm = [float("nan")] * 6
        else:
            fm = list(self._k[0] * dp / 1000.0) + list(self._k[1] * np.radians(dr))
        return ([time.perf_counter() - self._t0, self.tag]
                + list(cur.position) + list(cur.orientation.as_quat())
                + list(tgt.position) + list(tgt.orientation.as_quat())
                + [pos_err, ang_err, down_angle_deg(cur.orientation)]
                + list(dp) + list(dr) + fm
                + list(q) + list(dq) + list(tau))

    def _run(self) -> None:
        period = 1.0 / C.LOG_HZ
        k = 0
        t0 = time.perf_counter()
        while not self._stop.is_set():
            row = self._sample()
            if row is not None:
                self.rows.append(row)
            k += 1
            _sleep_until(t0 + k * period)

    def stop(self) -> str:
        self._stop.set()
        self._thread.join(timeout=2.0)
        csv_path = f"{self.base}.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            w.writerows(self.rows)
        with open(f"{self.base}.json", "w") as f:
            json.dump(self.meta, f, indent=2, default=str)
        return csv_path

    def column(self, name: str, tag: str | None = None) -> np.ndarray:
        i = self.COLS.index(name)
        rows = [row for row in self.rows if tag is None or row[1] == tag]
        return np.array([row[i] for row in rows], dtype=float)


# ----------------------------------------------------------------------------
# Script scaffolding
# ----------------------------------------------------------------------------
def add_common_args(ap: argparse.ArgumentParser, controller_switch: bool = True) -> None:
    g = ap.add_argument_group("common")
    g.add_argument("--log", metavar="NAME", default=None,
                   help="write data/logs/NAME_<mode>_<time>.csv (+ .json with controller parameters)")
    g.add_argument("--lin-speed", type=float, default=C.LIM["lin_speed"], help="m/s for ramps")
    g.add_argument("--ang-speed", type=float, default=C.LIM["ang_speed"], help="rad/s for ramps")
    g.add_argument("--yes", action="store_true", help="skip the real-robot confirmation prompt")
    if controller_switch:
        g.add_argument("--no-switch", action="store_true",
                       help="do not switch controllers (use whatever is already active)")


def confirm_real(args) -> None:
    if C.MODE != "real" or FAKE or getattr(args, "yes", False):
        return
    print("\n*** REAL ROBOT MODE ***")
    print(f"Safety box: {C.BOUNDS_MIN} .. {C.BOUNDS_MAX} m, max ramp speed {C.LIM['max_lin_speed']} m/s")
    print("Check: workspace clear, user stop in hand, FCI/brakes OK, low gains set.")
    if input("Type 'go' to continue: ").strip().lower() != "go":
        print("aborted.")
        sys.exit(1)


@contextmanager
def session(args, script: str, controller: str | None = C.CARTESIAN_CONTROLLER):
    """connect + confirm + switch controller + logger; always hold the pose on exit."""
    confirm_real(args)
    r = connect()
    lg = None
    try:
        if controller and not getattr(args, "no_switch", False):
            ensure_controller(r, controller)
        if args.log:
            lg = Logger(r, args.log, script=script)
        yield r, lg
    except KeyboardInterrupt:
        print("\n[lab] interrupted: holding current pose.")
    finally:
        hold(r)
        if lg:
            print(f"[lab] log written: {lg.stop()}")
        r.shutdown()
