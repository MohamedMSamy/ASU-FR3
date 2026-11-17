"""lab_fake.py  --  a pretend robot so you can rehearse the scripts with NO ROS and NO sim.

Enable with:  export CRISP_LAB_FAKE=1
It mimics the part of crisp_py.Robot that the scripts use.  The "plant" is a first-order
lag with a dead band (so the dead-band tests have something to find).  It is NOT a model
of the FR3 or of CRISP: use it to check your waypoint files and geometry, and to see
what the output and logs look like.  Tune it with CRISP_LAB_FAKE_DEADBAND_MM / _DEG.
"""
from __future__ import annotations

import os
import threading
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

import lab_config as C


class Pose:
    """Same fields as crisp_py.utils.geometry.Pose (position, orientation)."""

    def __init__(self, position, orientation):
        self.position = np.asarray(position, dtype=float)
        self.orientation = orientation

    def copy(self):
        return Pose(self.position.copy(), Rotation.from_quat(self.orientation.as_quat()))


class _Switcher:
    def __init__(self):
        self.active = "joint_trajectory_controller"

    def get_controller_list(self):
        names = ["joint_state_broadcaster", "pose_broadcaster", "twist_broadcaster",
                 C.CARTESIAN_CONTROLLER, C.JOINT_CONTROLLER, "joint_trajectory_controller",
                 "gravity_compensation"]
        out = []
        for n in names:
            st = "active" if (n.endswith("broadcaster") or n == self.active) else "inactive"
            out.append(SimpleNamespace(name=n, state=st, type="fake"))
        return out

    def switch_controller(self, name, controllers_that_should_be_active=None):
        self.active = name
        return True

    def _switch_controller(self, to_deactivate, to_activate):
        self.active = to_activate[-1]
        return True


class _Params:
    DEFAULTS = {"task.k_pos_x": 300.0, "task.k_pos_y": 300.0, "task.k_pos_z": 300.0,
                "task.k_rot_x": 60.0, "task.k_rot_y": 60.0, "task.k_rot_z": 60.0,
                "use_operational_space": True, "nullspace.stiffness": 2.0}

    def get_parameters(self, names):
        out = []
        for n in names:
            if n.startswith("nullspace.weights."):
                out.append(150.0)
            else:
                out.append(self.DEFAULTS.get(n))
        return out


class FakeRobot:
    def __init__(self):
        self.config = SimpleNamespace(joint_names=C.JOINT_NAMES, publish_frequency=C.PUBLISH_HZ, time_to_home=1.0)
        self.node = None
        self.controller_switcher_client = _Switcher()
        self.cartesian_controller_parameters_client = _Params()
        self.joint_controller_parameters_client = _Params()
        self._lab_effort = np.zeros(7)
        self.tau = 0.15                                                   # time constant [s]
        self.db_pos = float(os.environ.get("CRISP_LAB_FAKE_DEADBAND_MM", "1.0")) / 1000.0
        self.db_rot = np.radians(float(os.environ.get("CRISP_LAB_FAKE_DEADBAND_DEG", "1.0")))
        self.db_joint = 0.0005                                            # rad
        self._pose = Pose(C.READY_POS.copy(), C.DOWN)
        self._target = self._pose.copy()
        self._q = C.READY_Q.copy()
        self._tq = C.READY_Q.copy()
        self._dq = np.zeros(7)
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    # -- simulated plant ----------------------------------------------------
    def _run(self):
        dt = 0.005
        while not self._stop:
            k = dt / self.tau
            e = self._target.position - self._pose.position
            eff = np.where(np.abs(e) > self.db_pos, e - np.sign(e) * self.db_pos, 0.0)
            self._pose.position = self._pose.position + eff * k
            rv = (self._target.orientation * self._pose.orientation.inv()).as_rotvec()
            m = np.linalg.norm(rv)
            if m > self.db_rot:
                rv_eff = rv * (m - self.db_rot) / m
                self._pose.orientation = Rotation.from_rotvec(rv_eff * k) * self._pose.orientation
            ej = self._tq - self._q
            ejeff = np.where(np.abs(ej) > self.db_joint, ej - np.sign(ej) * self.db_joint, 0.0)
            newq = self._q + ejeff * k
            self._dq = (newq - self._q) / dt
            self._q = newq
            time.sleep(dt)

    # -- the crisp_py.Robot surface the scripts use ---------------------------
    @property
    def end_effector_pose(self):
        return self._pose.copy()

    @property
    def target_pose(self):
        return self._target.copy()

    @property
    def joint_values(self):
        return self._q.copy()

    @property
    def joint_velocities(self):
        return self._dq.copy()

    @property
    def target_joint(self):
        return self._tq.copy()

    def wait_until_ready(self, timeout=10.0, check_frequency=10.0):
        return None

    def set_target(self, position=None, pose=None):
        p = pose.copy() if pose is not None else self._target.copy()
        if position is not None:
            p.position = np.asarray(position, dtype=float)
        self._target = p.copy()

    def set_target_joint(self, q):
        self._tq = np.asarray(q, dtype=float).copy()

    @property
    def joint_trajectory_controller_client(self):
        robot = self

        class _J:
            def send_joint_config(self, names, q, T=5.0, blocking=True):
                robot.home(q)
        return _J()

    def home(self, home_config=None, blocking=True):
        q = C.READY_Q if home_config is None else np.asarray(home_config, dtype=float)
        self._tq = q.copy()
        self._target = Pose(C.READY_POS.copy(), C.DOWN)
        time.sleep(self.config.time_to_home if hasattr(self.config, "time_to_home") else 1.0)

    def shutdown(self):
        self._stop = True
