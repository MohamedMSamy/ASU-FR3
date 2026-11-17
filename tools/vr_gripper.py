"""Quest 2 trigger -> Franka Hand open/close, via the franka_gripper action servers.

Right index trigger (press_index): each press toggles the hand, closed <-> open.
Runs next to vr_teleop.py (it only reads q2r_right_hand_inputs, it never touches target_pose).
It talks to franka_gripper/grasp and franka_gripper/move directly, not to the crisp_py adapter,
because the adapter always grasps at 50 N with width 0; here width and force are parameters.

  grasp_width   hand width (m) to close to; set it to the peg diameter. 0.0 closes fully.
  grasp_force   N, kept low on purpose
  grasp_speed   m/s
  open_width    m, 0.08 is the fully open hand
  dry_run       true = only log what would be sent (use it in the sim: the MuJoCo model has no hand)

Live: ros2 param set /vr_gripper grasp_width 0.012
The node assumes the hand starts open; if it is closed at start, the first press does nothing visible.
"""

import rclpy
from franka_msgs.action import Grasp, Move
from quest2ros.msg import OVR2ROSInputs
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class VRGripper(Node):
    def __init__(self):
        super().__init__("vr_gripper")
        self.declare_parameter("grasp_width", 0.0)
        self.declare_parameter("grasp_force", 20.0)
        self.declare_parameter("grasp_speed", 0.05)
        self.declare_parameter("open_width", 0.08)
        self.declare_parameter("open_speed", 0.1)
        self.declare_parameter("epsilon_inner", 0.01)
        self.declare_parameter("epsilon_outer", 0.08)
        self.declare_parameter("trigger_threshold", 0.7)
        self.declare_parameter("min_interval", 0.6)  # s between accepted presses
        self.declare_parameter("dry_run", False)
        self.declare_parameter("gripper_namespace", "franka_gripper")

        ns = self.get_parameter("gripper_namespace").value
        self.grasp_client = ActionClient(self, Grasp, f"{ns}/grasp")
        self.move_client = ActionClient(self, Move, f"{ns}/move")

        self.closed = False
        self.busy = False
        self.was_pressed = False
        self.last_accept = self.get_clock().now()

        self.create_subscription(
            OVR2ROSInputs, "q2r_right_hand_inputs", self.on_inputs, qos_profile_system_default
        )
        self.get_logger().info("VR gripper ready. Index trigger toggles the hand (assumed open).")

    def on_inputs(self, msg: OVR2ROSInputs):
        pressed = msg.press_index > self.get_parameter("trigger_threshold").value
        rising = pressed and not self.was_pressed
        self.was_pressed = pressed
        if not rising:
            return
        now = self.get_clock().now()
        if (now - self.last_accept).nanoseconds * 1e-9 < self.get_parameter("min_interval").value:
            return
        if self.busy:
            self.get_logger().warn("Gripper still moving, press ignored.")
            return
        self.last_accept = now
        self.send_close() if not self.closed else self.send_open()

    def server_ready(self, client):
        if self.get_parameter("dry_run").value:
            return False
        if not client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                f"No action server '{client._action_name}'. Is franka_gripper_node running? "
                "(the sim has no hand: use dry_run:=true)"
            )
            return False
        return True

    def send_close(self):
        p = self.get_parameter
        self.get_logger().info(
            f"CLOSE: width {p('grasp_width').value} m, force {p('grasp_force').value} N"
        )
        self.closed = True
        if not self.server_ready(self.grasp_client):
            return
        goal = Grasp.Goal()
        goal.width = float(p("grasp_width").value)
        goal.speed = float(p("grasp_speed").value)
        goal.force = float(p("grasp_force").value)
        goal.epsilon.inner = float(p("epsilon_inner").value)
        goal.epsilon.outer = float(p("epsilon_outer").value)
        self.busy = True
        self.grasp_client.send_goal_async(goal).add_done_callback(self.on_goal)

    def send_open(self):
        p = self.get_parameter
        self.get_logger().info(f"OPEN: width {p('open_width').value} m")
        self.closed = False
        if not self.server_ready(self.move_client):
            return
        goal = Move.Goal()
        goal.width = float(p("open_width").value)
        goal.speed = float(p("open_speed").value)
        self.busy = True
        self.move_client.send_goal_async(goal).add_done_callback(self.on_goal)

    def on_goal(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Gripper goal rejected.")
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        res = future.result().result
        self.busy = False
        if res.success:
            self.get_logger().info("Gripper done.")
        else:
            self.get_logger().warn(f"Gripper failed: {res.error}")


def main(args=None):
    rclpy.init(args=args)
    node = VRGripper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
