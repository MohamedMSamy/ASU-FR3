"""RViz markers for the VR target: where the controller is told to go, and how far that is from the TCP.

Subscribes to target_pose (from vr_teleop.py) and the TF base -> fr3_hand_tcp, publishes a MarkerArray
on vr_target_markers (frame: base):
  - sphere + x/y/z axes (red/green/blue) at the target pose
  - a line from the TCP to the target, with the lead in mm and degrees as text
  - the safety box (LIMITS of scripts/lab_config.py for CRISP_LAB_MODE) as a translucent blue cube
  - colour: green while the clutch is held (q2r_right_hand_inputs press_middle), grey while it holds
    the last target, red if no target_pose arrived for 'stale_after' s
RViz (opens with the stack): Add > By topic > /vr_target_markers > MarkerArray, Fixed Frame = base.
Read-only: it publishes nothing the robot listens to.
"""

import math
import os
import sys

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from quest2ros.msg import OVR2ROSInputs
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:  # the safety box, drawn if scripts/lab_config.py can be imported (needs numpy + scipy)
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
    import lab_config as _C

    BOX = (_C.BOUNDS_MIN.tolist(), _C.BOUNDS_MAX.tolist())
except Exception:  # noqa: BLE001
    BOX = None

S = math.sqrt(0.5)


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class VRTargetViz(Node):
    def __init__(self):
        super().__init__("vr_target_viz")
        self.base_frame = self.declare_parameter("base_frame", "base").value
        self.tcp_frame = self.declare_parameter("tcp_frame", "fr3_hand_tcp").value
        self.axis_len = self.declare_parameter("axis_length", 0.06).value
        self.stale_after = self.declare_parameter("stale_after", 0.5).value
        self.clutch_threshold = self.declare_parameter("clutch_threshold", 0.5).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target = None
        self.target_time = None
        self.clutched = False

        self.create_subscription(PoseStamped, "target_pose", self.on_target, qos_profile_system_default)
        self.create_subscription(
            OVR2ROSInputs, "q2r_right_hand_inputs", self.on_inputs, qos_profile_system_default
        )
        self.pub = self.create_publisher(MarkerArray, "vr_target_markers", qos_profile_system_default)
        self.create_timer(0.05, self.step)
        self.get_logger().info("VR target visualization ready: add /vr_target_markers in RViz.")

    def on_target(self, msg: PoseStamped):
        self.target = msg.pose
        self.target_time = self.get_clock().now()

    def on_inputs(self, msg: OVR2ROSInputs):
        self.clutched = msg.press_middle > self.clutch_threshold

    def tcp(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.tcp_frame, rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation, t.transform.rotation

    def marker(self, mid, mtype, color):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "vr_target"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def box_marker(self):
        lo, hi = BOX
        m = self.marker(6, Marker.CUBE, (0.2, 0.5, 1.0, 0.12))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [(a + b) / 2 for a, b in zip(lo, hi)]
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = [b - a for a, b in zip(lo, hi)]
        return m

    def step(self):
        if BOX is not None:
            box = MarkerArray()
            box.markers.append(self.box_marker())
            self.pub.publish(box)
        if self.target is None:
            return
        age = (self.get_clock().now() - self.target_time).nanoseconds * 1e-9
        if age > self.stale_after:
            base_color = (0.9, 0.15, 0.15, 0.9)
        elif self.clutched:
            base_color = (0.1, 0.85, 0.2, 0.95)
        else:
            base_color = (0.6, 0.6, 0.6, 0.9)

        p, o = self.target.position, self.target.orientation
        q = (o.x, o.y, o.z, o.w)
        arr = MarkerArray()

        sph = self.marker(0, Marker.SPHERE, base_color)
        sph.pose = self.target
        sph.scale.x = sph.scale.y = sph.scale.z = 0.02
        arr.markers.append(sph)

        # local x, y, z axes: rotate the arrow's own x onto each axis
        for i, (qa, col) in enumerate(
            [
                ((0.0, 0.0, 0.0, 1.0), (1.0, 0.1, 0.1, 1.0)),
                ((0.0, 0.0, S, S), (0.1, 1.0, 0.1, 1.0)),  # +90 deg about z: x -> y
                ((0.0, -S, 0.0, S), (0.2, 0.3, 1.0, 1.0)),  # -90 deg about y: x -> z
            ],
            start=1,
        ):
            a = self.marker(i, Marker.ARROW, col)
            a.pose.position = p
            qq = q_mul(q, qa)
            a.pose.orientation.x, a.pose.orientation.y = qq[0], qq[1]
            a.pose.orientation.z, a.pose.orientation.w = qq[2], qq[3]
            a.scale.x, a.scale.y, a.scale.z = self.axis_len, 0.006, 0.006
            arr.markers.append(a)

        cur = self.tcp()
        if cur is not None:
            c, qc = cur
            d = (p.x - c.x, p.y - c.y, p.z - c.z)
            dist = math.sqrt(sum(x * x for x in d))
            dot = abs(q[0] * qc.x + q[1] * qc.y + q[2] * qc.z + q[3] * qc.w)
            ang = math.degrees(2.0 * math.acos(min(1.0, dot)))

            line = self.marker(4, Marker.LINE_STRIP, base_color)
            line.scale.x = 0.004
            line.points = [Point(x=c.x, y=c.y, z=c.z), Point(x=p.x, y=p.y, z=p.z)]
            arr.markers.append(line)

            txt = self.marker(5, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
            txt.pose.position.x, txt.pose.position.y = p.x, p.y
            txt.pose.position.z = p.z + 0.06
            txt.scale.z = 0.025
            txt.text = f"lead {dist * 1000:.0f} mm / {ang:.1f} deg"
            arr.markers.append(txt)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VRTargetViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
