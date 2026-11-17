"""MuJoCo as a stand-in for Isaac Sim's ROS 2 bridge.

It does what the Isaac side has to do for `./fr3 up isaac`, so the whole torque path can be tested without Isaac:
  * publishes sensor_msgs/JointState (position, velocity, effort = the torque applied) every physics step,
  * subscribes to the effort commands, holds the last command (zero-order hold) and applies it to the arm,
  * steps MuJoCo at a fixed rate with the same scene as the `mujoco` mode (FR3_SCENE, through the config overlay).
Topics and rate (environment, same names the isaac launch uses):
  ISAAC_JOINT_STATES_TOPIC (/isaac_joint_states), ISAAC_JOINT_COMMANDS_TOPIC (/isaac_joint_commands),
  BRIDGE_PHYSICS_HZ (1000), BRIDGE_REALTIME_FACTOR (1.0; 0.2 = one physics step takes 5 ms of wall time, like an Isaac Sim that
  runs at 0.2x real time).
Every 5 s it prints one `[bridge]` line with the achieved rates, the number of command messages with NaN, and the
number of late steps, which the internal Isaac checks read.

  ros2 run asu_fr3_bringup mujoco_isaac_bridge
"""
import math
import os
import threading
import time

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from asu_fr3_bringup import description, overlay

JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]


class Bridge(Node):
    def __init__(self):
        super().__init__("mujoco_isaac_bridge")
        hz = float(os.environ.get("BRIDGE_PHYSICS_HZ", "1000"))
        self.rtf = float(os.environ.get("BRIDGE_REALTIME_FACTOR", "1.0"))
        states_topic = os.environ.get("ISAAC_JOINT_STATES_TOPIC", "/isaac_joint_states")
        commands_topic = os.environ.get("ISAAC_JOINT_COMMANDS_TOPIC", "/isaac_joint_commands")

        eff = overlay.get_effective()
        scene = description.scene_name("isaac")
        self.model = mujoco.MjModel.from_xml_path(str(eff.file("models", scene)))
        self.model.opt.timestep = 1.0 / hz
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)   # keyframe "home"

        # joint name -> qpos / dof address, actuator index
        jid = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
        self.qadr = np.array([self.model.jnt_qposadr[j] for j in jid])
        self.dadr = np.array([self.model.jnt_dofadr[j] for j in jid])
        act_of_joint = {int(self.model.actuator_trnid[a, 0]): a for a in range(self.model.nu)}
        self.act = np.array([act_of_joint[j] for j in jid])

        self.cmd = np.zeros(len(JOINTS))        # last effort command (zero-order hold)
        self.lock = threading.Lock()
        self.n_cmd = 0
        self.n_nan = 0
        self.n_pub = 0
        self.n_late = 0

        # Isaac Sim's joint-state publisher is RELIABLE / VOLATILE (read with `ros2 topic info -v`, 2026-11-15); the plugin subscribes best effort
        self.pub = self.create_publisher(JointState, states_topic, QoSProfile(depth=10))
        self.create_subscription(JointState, commands_topic, self.on_command, QoSProfile(depth=10))
        self.create_timer(5.0, self.report)
        self.t_report = time.perf_counter()
        self.last_counts = (0, 0)
        self.get_logger().info(
            f"[bridge] {scene}, physics {hz:.0f} Hz (x{self.rtf:g} real time), states {states_topic}, commands {commands_topic}"
        )
        self.running = True
        self.thread = threading.Thread(target=self.physics_loop, daemon=True)
        self.thread.start()

    def on_command(self, msg):
        self.n_cmd += 1
        if len(msg.effort) < len(msg.name):
            return
        bad = False
        with self.lock:
            for name, effort in zip(msg.name, msg.effort):
                if name in JOINTS:
                    if math.isfinite(effort):
                        self.cmd[JOINTS.index(name)] = effort
                    else:
                        bad = True
        if bad:
            self.n_nan += 1

    def physics_loop(self):
        dt = self.model.opt.timestep
        wall_dt = dt / self.rtf          # wall time per physics step
        names = list(JOINTS)
        next_t = time.perf_counter()
        while self.running:
            with self.lock:
                tau = self.cmd.copy()
            self.data.ctrl[self.act] = tau
            mujoco.mj_step(self.model, self.data)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = self.data.qpos[self.qadr].tolist()
            msg.velocity = self.data.qvel[self.dadr].tolist()
            msg.effort = tau.tolist()
            self.pub.publish(msg)
            self.n_pub += 1
            next_t += wall_dt
            remaining = next_t - time.perf_counter()
            if remaining < 0:
                self.n_late += 1
                if remaining < -50 * wall_dt:       # far behind: drop the backlog instead of racing to catch up
                    next_t = time.perf_counter()
                continue
            if remaining > 0.0004:
                time.sleep(remaining - 0.0003)
            while time.perf_counter() < next_t:
                pass

    def report(self):
        now = time.perf_counter()
        span = now - self.t_report
        pub0, cmd0 = self.last_counts
        self.get_logger().info(
            f"[bridge] states {(self.n_pub - pub0) / span:.0f}/s, commands {(self.n_cmd - cmd0) / span:.0f}/s, "
            f"nan_command_msgs {self.n_nan}, late_steps {self.n_late}"
        )
        self.t_report, self.last_counts = now, (self.n_pub, self.n_cmd)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
