"""vr_teleop.py with ONE addition: the target position is clamped to the safety box LIMITS[mode] of scripts/lab_config.py.

Same node, same name (/vr_teleop, so ./fr3 scale still works), same parameters, same features as vr_teleop.py: grip clutch, scales, lead limits,
the B-button reset to home, the hand tilt limit (max_tilt_deg). The only differences:
  * every target_pose is clamped per axis to [BOUNDS_MIN, BOUNDS_MAX] before it is published. If the TCP is already outside the box on an axis, that axis
    may not move further out, but it may move back in (no jump towards the box when you engage the clutch).
  * the tilt limit shrinks near the box: it is max_tilt_deg (default 20) when the target is at least edge_margin (default 0.05 m) inside the box and falls
    linearly to edge_tilt_deg (default 8) at the box faces. At the edge of the workspace the hand may therefore tilt less (that is where a hand
    turned toward the robot base made the arm unstable).
The mode (sim or real) comes from CRISP_LAB_MODE, which the stack sets (./fr3 run / vr).
"""

import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import lab_config as C  # noqa: E402
from vr_teleop import VRTeleop  # noqa: E402


class ClampedPublisher:
    """Stands in for VRTeleop.pub: clamps the position of each PoseStamped, then publishes it."""

    def __init__(self, node, pub, lo, hi):
        self.node, self.pub, self.lo, self.hi = node, pub, lo, hi

    def publish(self, msg):
        p = msg.pose.position
        tcp = self.node.tcp_pose()
        vals = [p.x, p.y, p.z]
        cur = None if tcp is None else [tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z]
        hit = []
        for i, name in enumerate("xyz"):
            lo, hi = float(self.lo[i]), float(self.hi[i])
            if cur is not None:  # already outside: do not pull towards the box, just do not go further out
                lo, hi = min(lo, cur[i]), max(hi, cur[i])
            c = min(max(vals[i], lo), hi)
            if c != vals[i]:
                hit.append(name)
                vals[i] = c
        p.x, p.y, p.z = vals
        if hit:
            self.node.get_logger().warn(
                f"target clamped to the safety box on {','.join(hit)}", throttle_duration_sec=2.0
            )
        self.pub.publish(msg)


class ClampedVRTeleop(VRTeleop):
    def __init__(self):
        super().__init__()
        self.pub = ClampedPublisher(self, self.pub, C.BOUNDS_MIN, C.BOUNDS_MAX)
        self.declare_parameter("edge_margin", 0.05)  # m inside the box where the tilt limit starts to shrink
        self.declare_parameter("edge_tilt_deg", 8.0)  # tilt limit at the box faces
        self.get_logger().info(
            f"Target clamped to the {C.MODE} safety box: min {C.BOUNDS_MIN.round(3).tolist()} "
            f"max {C.BOUNDS_MAX.round(3).tolist()} (scripts/lab_config.py). Tilt limit "
            f"{self.get_parameter('max_tilt_deg').value} deg, {self.get_parameter('edge_tilt_deg').value} deg at the box faces."
        )
        home = self.get_parameter("home_pos").value
        if any(h < lo or h > hi for h, lo, hi in zip(home, C.BOUNDS_MIN, C.BOUNDS_MAX)):
            self.get_logger().warn(
                f"home_pos {home} is OUTSIDE the safety box: the reset will stop at the box edge. "
                "Change READY_POS in lab_config.py or the home_pos parameter."
            )

    def tilt_limit_deg(self, position):
        base = super().tilt_limit_deg(position)
        if base <= 0.0:
            return base
        margin = max(float(self.get_parameter("edge_margin").value), 1e-6)
        edge = float(self.get_parameter("edge_tilt_deg").value)
        d = min(min(p - lo, hi - p) for p, lo, hi in zip(position, C.BOUNDS_MIN, C.BOUNDS_MAX))
        d = max(d, 0.0)
        if d >= margin:
            return base
        return min(base, edge + (base - edge) * d / margin)


def main(args=None):
    rclpy.init(args=args)
    node = ClampedVRTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
