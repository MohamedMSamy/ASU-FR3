"""lab_config.py  --  EDIT ME.

Settings shared by every script in scripts/.  Nothing here talks to ROS,
so you can change it freely.  Scripts pick the "sim" or "real" column of LIMITS
from the CRISP_LAB_MODE environment variable, which ./fr3 run sets for you
(sim for mujoco and isaac, real for real).
"""
import os
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# ----------------------------------------------------------------------------
# Where things live
# ----------------------------------------------------------------------------
# ASU-FR3: inside the image this file may be loaded from /ws_override/config, so the root is taken from ASU_FR3_DIR
# (the baked defaults, /opt/asu_fr3).  Logs go to FR3_LOG_DIR and your own files (waypoint CSVs ...) to FR3_DATA_DIR,
# both on the host in ./data (mounted at /home/ros/data).  Example files shipped with the image are in examples/.
LAB_ROOT = Path(os.environ.get("ASU_FR3_DIR", str(Path(__file__).resolve().parent.parent)))
DATA_DIR = Path(os.environ.get("FR3_DATA_DIR", str(LAB_ROOT / "data")))
LOG_DIR = Path(os.environ.get("FR3_LOG_DIR", str(DATA_DIR / "logs")))
EXAMPLES_DIR = LAB_ROOT / "examples"


def find_data_file(name):
    """A relative file name means: ./data/<name> if it exists, else the shipped examples/<name>, else ./data/<name>."""
    p = Path(name)
    if p.is_absolute() or p.exists():
        return p
    for d in (DATA_DIR, EXAMPLES_DIR):
        if (d / p).exists():
            return d / p
    return DATA_DIR / p

# "sim" or "real".  Set by ./fr3 run (real only for the real mode).  Defaults to sim.
MODE = os.environ.get("CRISP_LAB_MODE", "sim")

# ----------------------------------------------------------------------------
# Names that must match controllers.yaml and the URDF (fr3 description)
# ----------------------------------------------------------------------------
JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
BASE_FRAME = "base"
TARGET_FRAME = "fr3_hand_tcp"
JOINT_STATE_TOPIC = "franka/joint_states"          # remapped by franka.launch.py
CARTESIAN_CONTROLLER = "cartesian_impedance_controller"
JOINT_CONTROLLER = "joint_impedance_controller"

# ----------------------------------------------------------------------------
# Reference poses
# ----------------------------------------------------------------------------
# Franka "ready" joint configuration (same as the srdf group_state "ready").
READY_Q = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])

# Hand pointing straight down.  Quaternion in scipy order (x, y, z, w).
DOWN = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])

# Default Cartesian start point for the tests (base frame, metres).
READY_POS = np.array([0.40, 0.00, 0.45])

# ----------------------------------------------------------------------------
# Safety limits, per mode.  The scripts REFUSE to command anything outside the
# box, and cap speeds.  Tighten the "real" column for your table/fixtures.
# ----------------------------------------------------------------------------
LIMITS = {
    "sim": dict(
        bounds_min=np.array([0.20, -0.40, 0.25]),
        bounds_max=np.array([0.65, 0.40, 0.75]),
        lin_speed=0.10,      # m/s   default speed for ramps
        ang_speed=0.50,      # rad/s
        max_lin_speed=0.30,  # hard cap, scripts refuse above this
        max_accel=1.0,       # m/s^2 refuse geometry above this (centripetal / speed-up)
        max_corner_jump=0.15,  # m/s  sudden velocity change allowed at polygon corners
    ),
    "real": dict(
        bounds_min=np.array([0.30, -0.38, 0.10]),
        bounds_max=np.array([0.54, 0.38, 0.55]),
        lin_speed=0.05,
        ang_speed=0.25,
        max_lin_speed=0.10,
        max_accel=0.4,
        max_corner_jump=0.05,
    ),
}
LIM = LIMITS.get(MODE, LIMITS["sim"])
BOUNDS_MIN = LIM["bounds_min"]
BOUNDS_MAX = LIM["bounds_max"]

SETTLE_S = 3.0      # seconds to wait after each waypoint before measuring
PUBLISH_HZ = 50.0   # how fast scripts stream targets (crisp_py publishes at 50 Hz)
LOG_HZ = 100.0      # CSV logging rate

# ----------------------------------------------------------------------------
# Controller parameters recorded next to every log (so you always know what
# gains a log was taken with).  Names are ROS parameter names of the
# cartesian_impedance_controller node.  Add any you care about.
# ----------------------------------------------------------------------------
SNAPSHOT_PARAMS = [
    "task.k_pos_x", "task.k_pos_y", "task.k_pos_z",
    "task.k_rot_x", "task.k_rot_y", "task.k_rot_z",
    "task.d_pos_x", "task.d_pos_y", "task.d_pos_z",
    "task.d_rot_x", "task.d_rot_y", "task.d_rot_z",
    "nullspace.stiffness", "nullspace.damping", "nullspace.max_tau",
    "use_operational_space", "use_friction", "use_coriolis_compensation",
    "use_gravity_compensation", "use_local_jacobian",
    "limit_error", "limit_torques", "max_delta_tau",
    "filter.target_pose", "filter.q", "filter.dq", "filter.output_torque",
    "variable_max_stiffness.translational", "variable_max_stiffness.rotational",
    "variable_stiffness.enabled",
]
