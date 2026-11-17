#!/usr/bin/env python3
"""16_gripper.py  --  open, close or toggle the Franka Hand (the verb is  ./fr3 gripper).

It reads the hand's width first, so "toggle" always does the opposite of what the hand is doing now, and "open" on an open
hand (or "close" on a closed one) does nothing and says so.

  ./fr3 gripper                      toggle: open -> close, closed -> open   (the default)
  ./fr3 gripper open                 open to --open-width (skipped if already open)
  ./fr3 gripper close                grasp (skipped if already closed)
  ./fr3 gripper close --width 0.012  close on something 12 mm wide, 20 N
  ./fr3 gripper move 0.04            go to a width, no grasp (the Move action)
  ./fr3 gripper status               print the width and whether the hand counts as open
  ./fr3 gripper close --dry-run      print what would be sent, send nothing
  ./fr3 run fake 16_gripper.py       rehearsal: a fake hand remembered in /tmp, no ROS

The hand only works in the real stack (franka_gripper_node with its action servers). In the mujoco mode a fake hand node publishes a constant
width (0.07 m) and no action servers exist: `status` works, open/close fail with "no action server". Isaac mode has no hand node at all. The hand width is the sum of the two finger positions on franka_gripper/joint_states: 0.08 m is fully open.
"Open" means width above --open-above (0.07 m, the same threshold crisp_py uses); anything below counts as closed, also a hand
that holds a part.

Parameters (default; what it does; what to use)
  --width W          0.0 m    target width of the GRASP. 0.0 closes completely; for a part give its width (a 12 mm peg: 0.012).
  --force F          20 N     grasp force. Low on purpose. The hand takes 70 N continuous; refused above 70.
  --speed V          0.05 m/s closing speed. 0.02 to 0.1 for parts; refused above 0.1.
  --epsilon-inner E  0.01 m   the grasp counts as successful if the fingers stop between  width - E  and  width + --epsilon-outer.
  --epsilon-outer E  0.08 m   (with width 0.0 and 0.08 any part up to the full opening is accepted; narrow them to detect a missed part.)
  --open-width W     0.08 m   width the OPEN goes to; 0.08 is fully open. A smaller value leaves the hand half open.
  --open-speed V     0.1 m/s  opening speed.
  --open-above W     0.07 m   width above which the hand counts as open (the toggle and the "already" check use it).
  --timeout T        15 s     how long to wait for the hand to finish.
  --namespace NS     franka_gripper   namespace of the action servers (franka_gripper/grasp, franka_gripper/move).
  --dry-run                   print only.
Keep fingers, cables and the workspace clear: the hand closes at --force without any check of what is between the fingers.
"""
import argparse
import os
import sys
import time

FORCE_MAX = 70.0
SPEED_MAX = 0.1
FAKE = os.environ.get("CRISP_LAB_FAKE", "0") == "1"
FAKE_FILE = "/tmp/fr3_fake_hand_width"


class FakeHand:
    """A hand that remembers its width in a file, so two runs of the script toggle."""

    def __init__(self):
        try:
            self.width = float(open(FAKE_FILE).read())
        except (OSError, ValueError):
            self.width = 0.08

    def read_width(self, timeout, fresh=False):
        return self.width

    def grasp(self, goal, timeout):
        self.width = max(goal["width"], 0.0)
        open(FAKE_FILE, "w").write(str(self.width))
        return True, ""

    def move(self, goal, timeout):
        self.width = goal["width"]
        open(FAKE_FILE, "w").write(str(self.width))
        return True, ""

    def close(self):
        pass


class RosHand:
    def __init__(self, ns):
        import rclpy
        from franka_msgs.action import Grasp, Move
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.rclpy, self.Grasp, self.Move = rclpy, Grasp, Move
        rclpy.init()
        self.node = rclpy.create_node("fr3_gripper_cli")
        self.width = None
        self.node.create_subscription(JointState, f"{ns}/joint_states", self._on_state, qos_profile_sensor_data)
        self.grasp_client = ActionClient(self.node, Grasp, f"{ns}/grasp")
        self.move_client = ActionClient(self.node, Move, f"{ns}/move")

    def _on_state(self, msg):
        if len(msg.position) >= 2:
            self.width = float(msg.position[0] + msg.position[1])

    def read_width(self, timeout, fresh=False):
        if fresh:
            self.width = None
        t0 = time.time()
        while self.width is None and time.time() - t0 < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.width

    def _send(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=3.0):
            return False, f"no action server {client._action_name}"
        fut = client.send_goal_async(goal)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False, "goal rejected"
        res = handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        if not res.done():
            handle.cancel_goal_async()
            return False, f"no result after {timeout:g} s (cancel sent)"
        r = res.result().result
        return bool(r.success), str(r.error)

    def grasp(self, goal, timeout):
        g = self.Grasp.Goal()
        g.width, g.speed, g.force = goal["width"], goal["speed"], goal["force"]
        g.epsilon.inner, g.epsilon.outer = goal["epsilon_inner"], goal["epsilon_outer"]
        return self._send(self.grasp_client, g, timeout)

    def move(self, goal, timeout):
        g = self.Move.Goal()
        g.width, g.speed = goal["width"], goal["speed"]
        return self._send(self.move_client, g, timeout)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", nargs="?", default="toggle", choices=["toggle", "open", "close", "move", "status"])
    ap.add_argument("value", nargs="?", type=float, help="width in m, only for: move")
    ap.add_argument("--width", type=float, default=0.0)
    ap.add_argument("--force", type=float, default=20.0)
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--epsilon-inner", type=float, default=0.01)
    ap.add_argument("--epsilon-outer", type=float, default=0.08)
    ap.add_argument("--open-width", type=float, default=0.08)
    ap.add_argument("--open-speed", type=float, default=0.1)
    ap.add_argument("--open-above", type=float, default=0.07)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--namespace", default="franka_gripper")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    if not 0.0 < a.force <= FORCE_MAX:
        sys.exit(f"--force {a.force:g} N refused: use 0 to {FORCE_MAX:g} N")
    if not 0.0 < a.speed <= SPEED_MAX or not 0.0 < a.open_speed <= 0.2:
        sys.exit(f"--speed above {SPEED_MAX:g} m/s (or --open-speed above 0.2) refused")
    if not 0.0 <= a.width <= 0.08 or not 0.0 <= a.open_width <= 0.08:
        sys.exit("widths must be between 0 and 0.08 m")
    if a.action == "move" and (a.value is None or not 0.0 <= a.value <= 0.08):
        sys.exit("usage: ./fr3 gripper move <width in m, 0 to 0.08>")

    hand = FakeHand() if FAKE else RosHand(a.namespace)
    try:
        w = hand.read_width(3.0)
        if w is None:
            sys.exit(f"no hand: nothing on {a.namespace}/joint_states in 3 s (stack not running, wrong mode, or Isaac mode, which has no hand node).")
        is_open = w > a.open_above
        print(f"hand width {w:.4f} m: {'open' if is_open else 'closed'} (open above {a.open_above:g} m)")
        action = a.action
        if action == "status":
            return
        if action == "toggle":
            action = "close" if is_open else "open"
        if action == "open" and is_open:
            print("already open: nothing sent.")
            return
        if action == "close" and not is_open:
            print("already closed: nothing sent.")
            return
        if action == "close":
            goal = dict(width=a.width, speed=a.speed, force=a.force, epsilon_inner=a.epsilon_inner, epsilon_outer=a.epsilon_outer)
            print(f"CLOSE: grasp width {a.width:g} m, {a.force:g} N, {a.speed:g} m/s, tolerance -{a.epsilon_inner:g} / +{a.epsilon_outer:g} m")
            send = hand.grasp
        else:
            to = a.value if action == "move" else a.open_width
            goal = dict(width=to, speed=a.open_speed)
            print(f"{'MOVE' if action == 'move' else 'OPEN'}: to {to:g} m at {a.open_speed:g} m/s")
            send = hand.move
        if a.dry_run:
            print("dry run: nothing sent.")
            return
        ok, err = send(goal, a.timeout)
        w2 = hand.read_width(1.0, fresh=True)
        print(("done" if ok else f"FAILED: {err}") + (f", width now {w2:.4f} m" if w2 is not None else ""))
        if not ok and "no action server" in err:
            print("(in the mujoco mode the hand is a fake that only publishes a width; the action servers exist in the real stack)")
        if not ok:
            sys.exit(1)
    finally:
        hand.close()


if __name__ == "__main__":
    main()
