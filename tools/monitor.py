#!/usr/bin/env python3
"""monitor.py  --  live terminal dashboard: forces, joint torques, errors, targets and who is publishing them.

Open it in a second terminal next to a running stack (it only LISTENS; it never publishes and never touches the robot):

  ./fr3 monitor                 # mujoco stack
  ./fr3 monitor real            # real stack (domain 101)
  ./fr3 monitor --once          # print one frame as text and exit (for a log or a quick look)
  q quits.

What it shows (refreshed 5 times a second)
  * rates of the topics it listens to, and the number of publishers (and their node names) on target_pose / target_joint /
    target_wrench / target_stiffness: more than one publisher means the controller IGNORES that topic, a publisher you do not
    recognise is the "ghost" (the stale target an inactive controller keeps)
  * the TCP pose, the target pose, the error per axis (target - current, base frame; position in mm, rotation in degrees),
    the tilt of the hand from straight down and its yaw
  * the spring force the controller asks for, F = K * error (N) and M = K_rot * rotation error (N m), computed from the gains it
    reads from the controller. Approximate: the controller works in the tool frame and clips the error (0.1 m / 0.5 rad)
  * target_wrench: the extra force somebody publishes to the controller (usually zero)
  * per joint: position, velocity, MEASURED torque (the effort of /franka/joint_states; on the real arm the Franka torque sensor,
    which includes what the robot itself does for gravity), the COMMANDED torque tau_cmd (what the controller sent to the hardware: the command interface
    of the introspection stream, same value as ./fr3 rec and --log write; the stream is on without any setting), and the external torque / wrench if the franka broadcasters run
    (they are not configured in our stack, so those lines say "not available")
  * warnings: torque above 80 % of the nominal limit, velocity above 80 %, no or several target publishers, stale joint states

Limits used for the bars (Franka FR3 datasheet values, for display only): torque 87 N m for joints 1 to 4 and 12 N m for
joints 5 to 7; velocity 2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26 rad/s.
"""
import argparse
import curses
import os
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

try:
    import lab_config as C
    JOINTS, JS_TOPIC = list(C.JOINT_NAMES), C.JOINT_STATE_TOPIC
    MODE, DOWN = C.MODE, C.DOWN
except Exception:  # noqa: BLE001  (monitor still works without the lab settings)
    JOINTS, JS_TOPIC, MODE = [f"fr3_joint{i}" for i in range(1, 8)], "franka/joint_states", "?"
    DOWN = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])

TAU_MAX = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
DQ_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
TARGET_TOPICS = ["/target_pose", "/target_joint", "/target_wrench", "/target_stiffness"]
GAIN_NAMES = ["task.k_pos_x", "task.k_pos_y", "task.k_pos_z", "task.k_rot_x", "task.k_rot_y", "task.k_rot_z"]
OPT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class Monitor(Node):
    def __init__(self):
        super().__init__("fr3_monitor")
        self.lock = threading.Lock()
        self.count, self.rate, self.last = {}, {}, {}
        self.q = self.dq = self.tau = None
        self.cur = self.tgt = self.twist = self.wrench_tgt = self.wrench_ext = self.tau_ext = None
        self.names, self.values, self.intro = {}, None, {}
        self.gains = None
        self.pubs = {t: [] for t in TARGET_TOPICS}
        self.controllers = None
        self._t_rate = time.time()

        self.create_subscription(JointState, JS_TOPIC, self.on_js, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "current_pose", lambda m: self.put("cur", m, "current_pose"), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "target_pose", lambda m: self.put("tgt", m, "target_pose"), OPT)
        self.create_subscription(TwistStamped, "current_twist", lambda m: self.put("twist", m, "current_twist"), qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, "target_wrench", lambda m: self.put("wrench_tgt", m, "target_wrench"), OPT)
        self.create_subscription(WrenchStamped, "external_wrench", lambda m: self.put("wrench_ext", m, "external_wrench"), OPT)
        self.create_subscription(JointState, "external_torques_joint_state", lambda m: self.put("tau_ext", m, "external_torques"), OPT)
        try:
            from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
            qn = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(StatisticsNames, "/controller_manager/introspection_data/names", self.on_names, qn)
            self.create_subscription(StatisticsValues, "/controller_manager/introspection_data/values", self.on_values, OPT)
        except Exception:  # noqa: BLE001
            pass
        self.param_cli = self.create_client(GetParameters, "/cartesian_impedance_controller/get_parameters")
        self.create_timer(1.0, self.slow)

    def tick(self, name):
        self.count[name] = self.count.get(name, 0) + 1
        self.last[name] = time.time()

    def put(self, attr, msg, name):
        with self.lock:
            setattr(self, attr, msg)
        self.tick(name)

    def on_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if not all(n in idx for n in JOINTS):
            return
        pick = lambda arr: np.array([arr[idx[n]] for n in JOINTS]) if len(arr) == len(m.name) else None  # noqa: E731
        with self.lock:
            self.q, self.dq, self.tau = pick(m.position), pick(m.velocity), pick(m.effort)
        self.tick("joint_states")

    def on_names(self, m):
        self.names[m.names_version] = list(m.names)

    def on_values(self, m):
        names = self.names.get(m.names_version)
        if names and len(names) == len(m.values):
            with self.lock:
                self.intro = dict(zip(names, m.values))
        self.tick("introspection")

    def slow(self):
        now = time.time()
        dt = max(now - self._t_rate, 1e-3)
        self._t_rate = now
        with self.lock:
            for k, c in self.count.items():
                prev = getattr(self, "_prev", {}).get(k, 0)
                self.rate[k] = (c - prev) / dt
            self._prev = dict(self.count)
        for t in TARGET_TOPICS:
            try:
                self.pubs[t] = sorted({i.node_name for i in self.get_publishers_info_by_topic(t)})
            except Exception:  # noqa: BLE001
                pass
        if self.param_cli.service_is_ready():
            req = GetParameters.Request()
            req.names = GAIN_NAMES
            fut = self.param_cli.call_async(req)
            fut.add_done_callback(self.on_gains)

    def on_gains(self, fut):
        try:
            vals = [parameter_value_to_python(v) for v in fut.result().values]
            if len(vals) == 6 and all(isinstance(v, float) for v in vals):
                self.gains = vals
        except Exception:  # noqa: BLE001
            pass


def bar(x, full, width=14):
    f = min(abs(x) / full, 1.0) if full else 0.0
    n = int(round(f * width))
    return "[" + "#" * n + "." * (width - n) + "]"


def pose_arrays(msg):
    p, o = msg.pose.position, msg.pose.orientation
    return np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w])


def tilt_yaw(rot):
    z = rot.apply([0.0, 0.0, 1.0])
    tilt = float(np.degrees(np.arccos(np.clip(-z[2], -1.0, 1.0))))
    yaw = float(np.degrees((rot * DOWN.inv()).as_euler("zyx")[0]))
    return tilt, yaw


def frame(mon):
    """The dashboard as a list of (text, level); level is '', 'head', 'warn' or 'bad'."""
    L = []
    add = lambda t, lv="": L.append((t, lv))  # noqa: E731
    now = time.time()
    with mon.lock:
        q, dq, tau = mon.q, mon.dq, mon.tau
        cur, tgt, wt, we, te = mon.cur, mon.tgt, mon.wrench_tgt, mon.wrench_ext, mon.tau_ext
        intro, gains = dict(mon.intro), mon.gains
        rate, last, pubs = dict(mon.rate), dict(mon.last), {k: list(v) for k, v in mon.pubs.items()}
    dom = os.environ.get("ROS_DOMAIN_ID", "?")
    add(f"FR3 monitor   mode {MODE}   ROS_DOMAIN_ID {dom}   {time.strftime('%H:%M:%S')}   (q = quit)", "head")
    r = lambda k: f"{rate.get(k, 0.0):6.0f} Hz"  # noqa: E731
    add(f"rates  joint_states {r('joint_states')}   current_pose {r('current_pose')}   target_pose {r('target_pose')}   "
        f"introspection {r('introspection')}")
    warn = []
    if now - last.get("joint_states", 0) > 0.5:
        warn.append(("NO JOINT STATES (stack not running, wrong domain, or the arm stopped)", "bad"))
    n_tp = len(pubs["/target_pose"])
    for t in TARGET_TOPICS:
        names = pubs[t]
        flag = ""
        if len(names) > 1:
            flag = "   <-- MORE THAN ONE: the controller IGNORES this topic"
            warn.append((f"{len(names)} publishers on {t}: {', '.join(names)}", "bad"))
        add(f"publishers {t:<18} {len(names)}  {', '.join(names) if names else '-'}{flag}", "bad" if flag else "")
    if n_tp == 0:
        warn.append(("nobody publishes target_pose", "warn"))

    add("")
    add("TCP / TARGET / ERROR (error = target - current, base frame)", "head")
    if cur is None:
        add("  no current_pose yet", "warn")
    else:
        pc, rc = pose_arrays(cur)
        tilt, yaw = tilt_yaw(rc)
        add(f"  TCP    x {pc[0]:+.4f}  y {pc[1]:+.4f}  z {pc[2]:+.4f} m     hand tilt from down {tilt:5.1f} deg   yaw {yaw:+6.1f} deg")
        if tgt is None:
            add("  no target_pose received")
        else:
            pt, rt = pose_arrays(tgt)
            dp = (pt - pc) * 1000.0
            dr = np.degrees((rt * rc.inv()).as_rotvec())
            age = now - last.get("target_pose", now)
            add(f"  target x {pt[0]:+.4f}  y {pt[1]:+.4f}  z {pt[2]:+.4f} m     (last message {age:4.1f} s ago)")
            lv = "bad" if np.linalg.norm(dp) > 50 else ("warn" if np.linalg.norm(dp) > 10 else "")
            add(f"  error  ex {dp[0]:+8.2f}  ey {dp[1]:+8.2f}  ez {dp[2]:+8.2f} mm   |e| {np.linalg.norm(dp):7.2f} mm", lv)
            add(f"         erx {dr[0]:+7.3f}  ery {dr[1]:+7.3f}  erz {dr[2]:+7.3f} deg  |e| {np.linalg.norm(dr):7.3f} deg", lv)
            if gains is not None:
                f = np.array(gains[:3]) * dp / 1000.0
                m = np.array(gains[3:]) * np.radians(dr)
                add(f"  spring force  Fx {f[0]:+7.2f}  Fy {f[1]:+7.2f}  Fz {f[2]:+7.2f} N   |F| {np.linalg.norm(f):6.2f} N"
                    f"        (K = {gains[0]:.0f}/{gains[1]:.0f}/{gains[2]:.0f} N/m)")
                add(f"  spring torque Mx {m[0]:+7.3f}  My {m[1]:+7.3f}  Mz {m[2]:+7.3f} N m |M| {np.linalg.norm(m):6.3f} N m"
                    f"      (K_rot = {gains[3]:.0f}/{gains[4]:.0f}/{gains[5]:.0f} N m/rad)")
            else:
                add("  spring force: gains not read yet")
    if wt is not None:
        w = wt.wrench
        add(f"  target_wrench  F {w.force.x:+.2f} {w.force.y:+.2f} {w.force.z:+.2f} N   M {w.torque.x:+.3f} {w.torque.y:+.3f} {w.torque.z:+.3f} N m")
    else:
        add("  target_wrench  none")
    if we is not None:
        w = we.wrench
        add(f"  external wrench  F {w.force.x:+.2f} {w.force.y:+.2f} {w.force.z:+.2f} N   M {w.torque.x:+.3f} {w.torque.y:+.3f} {w.torque.z:+.3f} N m")
    else:
        add("  external wrench  not available")

    add("")
    add("JOINTS", "head")
    add(f"  {'joint':<5}{'q [deg]':>10}{'dq [rad/s]':>12}{'tau [N m]':>11}  {'torque / limit':<22}{'cmd tau':>10}{'ext tau':>10}", "head")
    if q is None or tau is None:
        add("  no joint states", "warn")
    else:
        for i, name in enumerate(JOINTS):
            ratio = abs(tau[i]) / TAU_MAX[i]
            vr = abs(dq[i]) / DQ_MAX[i]
            lv = "bad" if (ratio > 1.0 or vr > 1.0) else ("warn" if (ratio > 0.8 or vr > 0.8) else "")
            if lv:
                warn.append((f"{name}: torque {ratio * 100:.0f} % / velocity {vr * 100:.0f} % of nominal limit", lv))
            task = intro.get(f"command_interface.{JOINTS[i]}/effort")        # commanded torque (the stream's own name, see lab_columns.py)
            task = f"{task:+.2f}" if task is not None else "n/a"
            ext = f"{te.effort[i]:+.2f}" if te is not None and len(te.effort) > i else "n/a"
            add(f"  {'j' + str(i + 1):<5}{np.degrees(q[i]):>+10.2f}{dq[i]:>+12.3f}{tau[i]:>+11.2f}  "
                f"{bar(tau[i], TAU_MAX[i])} {ratio * 100:4.0f}%{task:>10}{ext:>10}", lv)
    add("")
    add("WARNINGS", "head")
    if warn:
        for t, lv in warn:
            add("  ! " + t, lv)
    else:
        add("  none")
    return L


def run_curses(mon):
    def loop(scr):
        curses.curs_set(0)
        scr.nodelay(True)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_YELLOW, -1)
        curses.init_pair(2, curses.COLOR_RED, -1)
        curses.init_pair(3, curses.COLOR_CYAN, -1)
        attr = {"": 0, "warn": curses.color_pair(1), "bad": curses.color_pair(2) | curses.A_BOLD, "head": curses.color_pair(3) | curses.A_BOLD}
        while True:
            if scr.getch() in (ord("q"), ord("Q")):
                return
            scr.erase()
            h, w = scr.getmaxyx()
            for y, (text, lv) in enumerate(frame(mon)[: h - 1]):
                try:
                    scr.addstr(y, 0, text[: w - 1], attr[lv])
                except curses.error:
                    pass
            scr.refresh()
            time.sleep(0.2)

    try:
        curses.wrapper(loop)
    finally:
        try:
            curses.curs_set(1)
        except curses.error:
            pass
        sys.stdout.write("\x1b[?25h")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", action="store_true", help="print one frame as text after 3 s and exit")
    a = ap.parse_args()
    rclpy.init()
    mon = Monitor()
    spin = threading.Thread(target=rclpy.spin, args=(mon,), daemon=True)
    spin.start()
    try:
        if a.once or not sys.stdout.isatty():
            time.sleep(3.0)
            print("\n".join(t for t, _ in frame(mon)))
        else:
            run_curses(mon)
    except KeyboardInterrupt:
        pass
    finally:
        mon.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
