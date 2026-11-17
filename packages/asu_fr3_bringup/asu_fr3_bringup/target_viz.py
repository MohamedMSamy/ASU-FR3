"""RViz helper: draws the target against the actual TCP pose.

Subscribes to `current_pose` (pose_broadcaster) and `target_pose` (what crisp_py and the goal marker publish) and publishes
a MarkerArray on `fr3_target_markers`: a sphere and axes at the target, a line from the TCP to it, and a text with the
position error in mm and the orientation error in degrees. Started by the launch when RViz is on (--rviz).

  ros2 run asu_fr3_bringup target_viz
"""
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray


def _angle_between(q1, q2):
    """Angle in degrees between two orientations given as geometry_msgs quaternions."""
    dot = abs(q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w)
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


class TargetViz(Node):
    def __init__(self):
        super().__init__("target_viz")
        self.current = None
        self.target = None
        self.create_subscription(PoseStamped, "current_pose", lambda m: setattr(self, "current", m), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "target_pose", lambda m: setattr(self, "target", m), qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, "fr3_target_markers", 10)
        # Where the label sits relative to the target (base frame, metres). The default puts it in front of the robot
        # (towards the default RViz camera, +x) and slightly up, so it is not hidden inside the robot model.
        self.declare_parameter("label_offset", [0.20, 0.0, 0.10])
        self.create_timer(0.05, self.tick)

    def marker(self, mid, mtype, frame):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "fr3_target"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def tick(self):
        if self.current is None or self.target is None:
            return
        frame = self.target.header.frame_id or "base"
        cp, tp = self.current.pose, self.target.pose
        arr = MarkerArray()
        sphere = self.marker(0, Marker.SPHERE, frame)
        sphere.pose = tp
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.02
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.5, 0.0, 0.9
        line = self.marker(1, Marker.LINE_STRIP, frame)
        line.scale.x = 0.004
        line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.9, 0.1, 1.0
        line.points = [Point(x=cp.position.x, y=cp.position.y, z=cp.position.z),
                       Point(x=tp.position.x, y=tp.position.y, z=tp.position.z)]
        err_mm = 1000.0 * math.dist((cp.position.x, cp.position.y, cp.position.z), (tp.position.x, tp.position.y, tp.position.z))
        err_deg = _angle_between(cp.orientation, tp.orientation)
        text = self.marker(2, Marker.TEXT_VIEW_FACING, frame)
        off = self.get_parameter("label_offset").value
        text.pose.position.x = tp.position.x + off[0]
        text.pose.position.y = tp.position.y + off[1]
        text.pose.position.z = tp.position.z + off[2]
        text.scale.z = 0.04
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        # No spaces on purpose: RViz gives a space in a text marker a fixed width that does not shrink with the font,
        # so "0.2 mm" came out as words almost a metre apart.
        text.text = f"{err_mm:.1f}mm|{err_deg:.1f}deg"
        arr.markers = [sphere, line, text]
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = TargetViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
