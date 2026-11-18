"""Column names and the introspection tap shared by `./fr3 rec` (tools/rec_introspection.py) and the `--log` logger (lab_common.Logger).

Both write the same names for the same values, so a `--log` file and a `rec` file of the same test can be laid side by side:
  qN, dqN        measured position [rad] and speed [rad/s] of joint N (1 to 7)
  tau_measN      REPORTED torque [N m] (real arm: Franka's measured joint torque, gravity included)
  tau_cmdN       COMMANDED torque [N m]: what the controller sent to the hardware
  cmd_limitedN   1 when that command was clipped to a limit
  cycles_<controller> / missed_<controller>   control cycles run / missed
  rec_*          statistics of the introspection stream itself
  pos_x..z, quat_x..w, tgt_pos_x..z, tgt_quat_x..w   TCP pose (/current_pose) and the target pose (/target_pose) [m, quaternion]
  vr_pos_x..z, vr_quat_x..w   the raw headset hand pose (q2r_right_hand_pose, before scaling and clutch); vr_clutch = press_middle,
                 vr_trigger = press_index (NaN while no headset is connected)
  anything else  kept under its controller-manager name (the cartesian_impedance_controller.* columns, if enable_introspection is on)
"""
import re
import threading

NAMES_TOPIC = "/controller_manager/introspection_data/names"
VALUES_TOPIC = "/controller_manager/introspection_data/values"


def friendly(col: str) -> str:
    """Controller-manager column name -> short name (unknown names are returned unchanged)."""
    m = re.fullmatch(r"state_interface\.fr3_joint(\d)/(position|velocity|effort)", col)
    if m:
        return {"position": "q", "velocity": "dq", "effort": "tau_meas"}[m.group(2)] + m.group(1)
    m = re.fullmatch(r"command_interface\.fr3_joint(\d)/effort", col)
    if m:
        return "tau_cmd" + m.group(1)
    m = re.fullmatch(r"command_interface\.fr3_joint(\d)/effort\.is_limited", col)
    if m:
        return "cmd_limited" + m.group(1)
    m = re.fullmatch(r"(.+)\.(total|failed)_triggers", col)
    if m:
        return ("cycles_" if m.group(2) == "total" else "missed_") + m.group(1)
    m = re.fullmatch(r"topic_stats\.~/introspection_data\.(.+)", col)
    if m:
        return "rec_" + m.group(1)
    return col


class IntrospectionTap:
    """Keeps the latest introspection sample (friendly name -> value) of a running stack. Subscribes on the given rclpy node."""

    def __init__(self, node):
        from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self._lock = threading.Lock()
        self._names = {}          # names_version -> friendly names
        self._latest = {}
        self._stamp = None
        qn = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qv = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(StatisticsNames, NAMES_TOPIC, self._on_names, qn)
        node.create_subscription(StatisticsNames, NAMES_TOPIC, self._on_names, 10)
        node.create_subscription(StatisticsValues, VALUES_TOPIC, self._on_values, qv)

    def _on_names(self, m):
        self._names[m.names_version] = [friendly(n) for n in m.names]

    def _on_values(self, m):
        names = self._names.get(m.names_version)
        if names and len(names) == len(m.values):
            d = dict(zip(names, m.values))
            with self._lock:
                self._latest, self._stamp = d, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def snapshot(self):
        """(stamp, {name: value}) of the newest sample; (None, {}) if nothing has arrived."""
        with self._lock:
            return self._stamp, dict(self._latest)


class PoseTap:
    """Sample-and-hold of the pose topics, so a row of a 1 kHz recording carries the newest TCP / target / VR values.

    `names` picks the groups: "tcp" (pos_*, quat_* from /current_pose), "target" (tgt_pos_*, tgt_quat_* from /target_pose),
    "vr" (vr_*: the raw headset hand pose and the two buttons). Values not received yet are NaN. snapshot() returns the same tuple
    object until a new message arrives, so a long recording shares it between rows (small memory).
    """
    NAN = float("nan")

    def __init__(self, node, names=("tcp", "target", "vr")):
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data, qos_profile_system_default
        self.columns = []
        self._vals = {}
        self._cache = None
        self._order = []
        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        def pose_group(prefix, topic, qos):
            cols = [f"{prefix}pos_{a}" for a in "xyz"] + [f"{prefix}quat_{a}" for a in "xyzw"]
            self._order.extend(cols)
            node.create_subscription(PoseStamped, topic, lambda m: self._on_pose(cols, m), qos)

        if "tcp" in names:
            pose_group("", "/current_pose", qos_profile_sensor_data)
        if "target" in names:
            pose_group("tgt_", "/target_pose", best)
        if "vr" in names:
            pose_group("vr_", "q2r_right_hand_pose", qos_profile_system_default)
            self._order.extend(["vr_clutch", "vr_trigger"])
            try:
                from quest2ros.msg import OVR2ROSInputs
                node.create_subscription(OVR2ROSInputs, "q2r_right_hand_inputs", self._on_inputs, qos_profile_system_default)
            except Exception:  # noqa: BLE001  (no quest2ros in this environment: the button columns stay NaN)
                pass
        self.columns = list(self._order)
        self._vals = {c: self.NAN for c in self._order}

    def _on_pose(self, cols, m):
        p, o = m.pose.position, m.pose.orientation
        self._vals.update(zip(cols, (p.x, p.y, p.z, o.x, o.y, o.z, o.w)))
        self._cache = None

    def _on_inputs(self, m):
        self._vals["vr_clutch"], self._vals["vr_trigger"] = float(m.press_middle), float(m.press_index)
        self._cache = None

    def snapshot(self):
        """Values in the order of `columns` (a tuple, shared until something new arrives)."""
        c = self._cache
        if c is None:
            c = self._cache = tuple(self._vals[k] for k in self._order)
        return c
