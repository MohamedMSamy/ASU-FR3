"""Clutch-based Quest 2 teleop: hand motion -> CRISP target_pose (position + orientation).

Hold the right grip (press_middle) to move. On press, the TCP pose and the hand pose are
latched; while held, the target is
    position    = tcp_pos_at_press + pos_scale * (hand_pos - hand_pos_at_press)
    orientation = rotvec_scaled(rot_scale, hand_rot * hand_rot_at_press^-1) * tcp_rot_at_press
(the hand's rotation since the press, scaled in axis-angle, applied in the base frame).
rot_scale 0 keeps the orientation fixed. On release the target holds at its last value.
Both scales can be changed live: ./fr3 scale --pos 0.5 --rot 0.3.
Run with the sim/robot controller already up.

Also in this node (vr_teleop_clamped.py adds ONLY the safety-box clamp on top of it):
  * Reset to home: press the right B button (button_upper; home_button:=lower for A) with the grip released. The target starts at the actual TCP,
    rises to home height, then moves to home_pos (READY_POS of lab_config.py) with the hand pointing DOWN, at home_speed (m/s) / home_ang_speed
    (rad/s). Pressing the button again, or the grip, cancels and holds.
  * Tilt limit: the hand may not tilt more than max_tilt_deg (default 20, 0 = off) away from straight down, whatever the hand does. The target
    orientation is brought back to the cone of that angle (same direction of tilt) before the lead limit is applied. Live:
    ros2 param set /vr_teleop max_tilt_deg 15. Why: with the TCP at the edge of the workspace, a hand tilted toward the robot base (or far
    sideways) is where the arm becomes unstable.
"""

import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from quest2ros.msg import OVR2ROSInputs
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Home pose and speeds from scripts/lab_config.py (on PYTHONPATH inside the container); conservative fallbacks if it cannot be imported.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
try:
    import lab_config as _C

    HOME_POS = [float(v) for v in _C.READY_POS]
    HOME_Q = tuple(float(v) for v in _C.DOWN.as_quat())  # x, y, z, w
    HOME_SPEED, HOME_ANG_SPEED = float(_C.LIM["lin_speed"]), float(_C.LIM["ang_speed"])
except Exception:  # noqa: BLE001
    HOME_POS, HOME_Q, HOME_SPEED, HOME_ANG_SPEED = [0.40, 0.0, 0.45], (1.0, 0.0, 0.0, 0.0), 0.05, 0.25


def q_mul(a, b):
    """Hamilton product of quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def q_to_rotvec(q):
    """Axis * angle (rad), angle in [0, pi]."""
    q = q_norm(q)
    if q[3] < 0.0:
        q = tuple(-c for c in q)
    s = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(s, q[3])
    return (q[0] / s * angle, q[1] / s * angle, q[2] / s * angle)


def q_from_rotvec(v):
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (v[0] * s, v[1] * s, v[2] * s, math.cos(angle / 2.0))


def q_rotate(q, v):
    """Rotate vector v by the quaternion q (x, y, z, w)."""
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    )


def limit_tilt(q, max_rad):
    """Bring the tool z axis to at most max_rad from straight down (same tilt direction). Returns (q, changed)."""
    z = q_rotate(q, (0.0, 0.0, 1.0))
    theta = math.acos(max(-1.0, min(1.0, -z[2])))
    if theta <= max_rad:
        return q, False
    n = math.hypot(z[0], z[1])
    u = (z[0] / n, z[1] / n) if n > 1e-9 else (1.0, 0.0)
    zc = (math.sin(max_rad) * u[0], math.sin(max_rad) * u[1], -math.cos(max_rad))
    axis = (z[1] * zc[2] - z[2] * zc[1], z[2] * zc[0] - z[0] * zc[2], z[0] * zc[1] - z[1] * zc[0])
    s = math.sqrt(sum(a * a for a in axis))
    if s < 1e-9:
        return q, False
    ang = math.atan2(s, sum(a * b for a, b in zip(z, zc)))
    return q_norm(q_mul(q_from_rotvec(tuple(a / s * ang for a in axis)), q)), True


class VRTeleop(Node):
    def __init__(self):
        super().__init__("vr_teleop")
        # Position and rotation have separate gains; rot_scale 0 keeps the TCP orientation fixed.
        self.declare_parameter("pos_scale", 0.5)
        self.declare_parameter("rot_scale", 0.5)
        # Max angle (rad) the target orientation may lead the TCP orientation.
        self.max_rot_lead = self.declare_parameter("max_rot_lead", 0.5).value
        self.base_frame = self.declare_parameter("base_frame", "base").value
        self.tcp_frame = self.declare_parameter("tcp_frame", "fr3_hand_tcp").value
        self.rate = self.declare_parameter("publish_frequency", 50.0).value
        self.clutch_threshold = self.declare_parameter("clutch_threshold", 0.5).value
        # Max distance (m) the target may lead the TCP; bounds a tracking glitch.
        self.max_lead = self.declare_parameter("max_lead", 0.15).value
        # Hand pose older than this (s) counts as lost tracking and drops the clutch.
        self.stale_after = self.declare_parameter("stale_after", 0.3).value
        # Largest tilt of the hand away from straight down [deg]; 0 switches the limit off. Read every step (live).
        self.declare_parameter("max_tilt_deg", 20.0)
        # Reset to home (see the module docstring).
        self.declare_parameter("home_button", "upper")  # "upper" = B/Y, "lower" = A/X
        self.declare_parameter("home_pos", list(HOME_POS))
        self.declare_parameter("home_speed", HOME_SPEED)  # m/s
        self.declare_parameter("home_ang_speed", HOME_ANG_SPEED)  # rad/s
        self.home_q = HOME_Q
        self.homing = False
        self.home_path = []
        self.btn_was = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.hand = None  # latest hand position (x, y, z)
        self.hand_time = None
        self.clutched = False
        self.hand0 = None
        self.hand_q = None  # latest hand orientation (x, y, z, w)
        self.hand_q0 = None
        self.tcp0 = None  # PoseStamped latched at clutch press
        self.target = None  # PoseStamped, held when clutch is released

        self.create_subscription(
            PoseStamped, "q2r_right_hand_pose", self.on_pose, qos_profile_system_default
        )
        self.create_subscription(
            OVR2ROSInputs, "q2r_right_hand_inputs", self.on_inputs, qos_profile_system_default
        )
        self.pub = self.create_publisher(
            PoseStamped, "target_pose", qos_profile=qos_profile_system_default
        )
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(
            f"VR teleop ready. Hold right grip to move. Hand tilt limit {self.get_parameter('max_tilt_deg').value} deg from straight down. "
            f"Reset to home: press the {self.get_parameter('home_button').value} button (again = cancel, grip = cancel), "
            f"home {self.get_parameter('home_pos').value}, hand down."
        )

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.hand = (p.x, p.y, p.z)
        o = msg.pose.orientation
        self.hand_q = q_norm((o.x, o.y, o.z, o.w))
        self.hand_time = self.get_clock().now()

    def tilt_limit_deg(self, position):
        """The tilt limit [deg] at this target position; subclasses may make it depend on where the target is."""
        return float(self.get_parameter("max_tilt_deg").value)

    def on_inputs(self, msg: OVR2ROSInputs):
        btn = msg.button_upper if self.get_parameter("home_button").value == "upper" else msg.button_lower
        rising = bool(btn) and not self.btn_was
        self.btn_was = bool(btn)
        if rising:
            self.toggle_home()
        if self.homing and msg.press_middle > self.clutch_threshold:
            self.cancel_home("grip pressed")
        pressed = msg.press_middle > self.clutch_threshold
        if pressed and not self.clutched:
            self.engage()
        elif not pressed and self.clutched:
            self.clutched = False
            self.get_logger().info("Clutch released, holding target.")

    def tcp_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"No TCP transform: {ex}", throttle_duration_sec=1.0)
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation
        return pose

    def toggle_home(self):
        if self.homing:
            self.cancel_home("button pressed again")
            return
        if self.clutched:
            self.get_logger().warn("Release the grip first, then press the reset button.")
            return
        tcp = self.tcp_pose()
        if tcp is None:
            self.get_logger().warn("Cannot reset: no TCP transform.")
            return
        home = self.get_parameter("home_pos").value
        c = tcp.pose.position
        # first rise to home height (never sweep sideways low over the table), then go home
        self.home_path = [(c.x, c.y, max(c.z, home[2])), tuple(home)]
        self.target = tcp  # start from where the TCP actually is: no lead left over
        self.homing = True
        self.get_logger().info(f"Resetting to home {home} at {self.get_parameter('home_speed').value} m/s.")

    def cancel_home(self, why):
        if self.homing:
            self.homing = False
            self.get_logger().info(f"Reset cancelled ({why}); holding the current target.")

    def advance_home(self):
        dt = 1.0 / self.rate
        pos_step = self.get_parameter("home_speed").value * dt
        ang_step = self.get_parameter("home_ang_speed").value * dt
        t = self.target.pose
        wp = self.home_path[0]
        last = len(self.home_path) == 1  # decided BEFORE any pop: the finish test must be about the real home, not the lift point
        v = (wp[0] - t.position.x, wp[1] - t.position.y, wp[2] - t.position.z)
        dist = sum(x * x for x in v) ** 0.5
        if dist <= pos_step:
            t.position.x, t.position.y, t.position.z = wp
            if not last:
                self.home_path.pop(0)
        else:
            k = pos_step / dist
            t.position.x += v[0] * k
            t.position.y += v[1] * k
            t.position.z += v[2] * k
        o = t.orientation
        q = (o.x, o.y, o.z, o.w)
        rv = q_to_rotvec(q_mul(self.home_q, q_conj(q)))
        ang = sum(x * x for x in rv) ** 0.5
        if ang <= ang_step:
            q = self.home_q
        else:
            q = q_norm(q_mul(q_from_rotvec(tuple(x * ang_step / ang for x in rv)), q))
        o.x, o.y, o.z, o.w = q
        if last and dist <= pos_step and ang <= ang_step:
            self.homing = False
            self.get_logger().info("Home reached, holding.")

    def hand_is_fresh(self):
        if self.hand is None:
            return False
        age = (self.get_clock().now() - self.hand_time).nanoseconds * 1e-9
        return age < self.stale_after

    def engage(self):
        tcp = self.tcp_pose()
        if tcp is None or not self.hand_is_fresh():
            self.get_logger().warn("Cannot engage: need TCP transform and fresh hand pose.")
            return
        self.tcp0 = tcp
        self.hand0 = self.hand
        self.hand_q0 = self.hand_q
        if self.target is None:
            self.target = tcp
        self.clutched = True
        self.get_logger().info("Clutch engaged.")

    def step(self):
        if self.homing:
            self.advance_home()
        if self.clutched and not self.hand_is_fresh():
            self.clutched = False
            self.get_logger().warn("Hand tracking stale, dropping clutch.")

        if self.clutched:
            # re-read each step so `ros2 param set` / ./fr3 scale works live
            pos_scale = self.get_parameter("pos_scale").value
            rot_scale = self.get_parameter("rot_scale").value
            d = [pos_scale * (h - h0) for h, h0 in zip(self.hand, self.hand0)]
            tgt = PoseStamped()
            tgt.header.frame_id = self.base_frame
            o0 = self.tcp0.pose.orientation
            q_t0 = (o0.x, o0.y, o0.z, o0.w)
            hand_delta = q_mul(self.hand_q, q_conj(self.hand_q0))
            rv = q_to_rotvec(hand_delta)
            q_t = q_norm(q_mul(q_from_rotvec(tuple(rot_scale * c for c in rv)), q_t0))
            p0 = self.tcp0.pose.position
            tgt.pose.position.x = p0.x + d[0]
            tgt.pose.position.y = p0.y + d[1]
            tgt.pose.position.z = p0.z + d[2]

            # Hand tilt limit (before the lead limit, so the arm is only ever asked for a small step toward the cone).
            max_tilt = self.tilt_limit_deg((tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z))
            if max_tilt > 0.0:
                q_t, limited = limit_tilt(q_t, math.radians(max_tilt))
                if limited:
                    self.get_logger().warn(
                        f"hand tilt limited to {max_tilt:.0f} deg from straight down", throttle_duration_sec=2.0
                    )

            # Keep the target within max_lead of the actual TCP.
            tcp = self.tcp_pose()
            if tcp is not None:
                c = tcp.pose.position
                v = [
                    tgt.pose.position.x - c.x,
                    tgt.pose.position.y - c.y,
                    tgt.pose.position.z - c.z,
                ]
                n = sum(x * x for x in v) ** 0.5
                if n > self.max_lead:
                    k = self.max_lead / n
                    tgt.pose.position.x = c.x + v[0] * k
                    tgt.pose.position.y = c.y + v[1] * k
                    tgt.pose.position.z = c.z + v[2] * k

                # Same for orientation: cap the angle between target and actual TCP.
                oc = tcp.pose.orientation
                q_c = (oc.x, oc.y, oc.z, oc.w)
                lead = q_to_rotvec(q_mul(q_t, q_conj(q_c)))
                angle = math.sqrt(sum(x * x for x in lead))
                if angle > self.max_rot_lead:
                    k = self.max_rot_lead / angle
                    q_t = q_norm(q_mul(q_from_rotvec(tuple(k * x for x in lead)), q_c))

            tgt.pose.orientation.x, tgt.pose.orientation.y = q_t[0], q_t[1]
            tgt.pose.orientation.z, tgt.pose.orientation.w = q_t[2], q_t[3]
            self.target = tgt

        if self.target is None:
            return  # never engaged: publish nothing, controller keeps its own target
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.target)


def main(args=None):
    rclpy.init(args=args)
    node = VRTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
