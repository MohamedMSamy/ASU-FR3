#!/usr/bin/env python3
"""09_rviz_goal.py  --  drag a marker in RViz, the hand follows (position, yaw and optional tilt).

By default the hand points straight down and you can move it (arrows = x / y / z, square = horizontal plane)
and turn it about the vertical (ring).  Right-click the marker for a menu:
  * "Tilt rings"   shows two more rings (tilt about x and about y).  Off by default to keep the marker uncluttered.
  * "Level hand"   removes the tilt again (keeps position and yaw).

The script is the ONLY thing publishing the target, and it goes through the same safety code as the other scripts:
  * the goal is clamped to the safety box in lab_config.py (the marker snaps back to the box edge)
  * the tilt of the hand away from straight down is limited to --max-tilt degrees (the marker snaps back)
  * the target moves towards the goal at --lin-speed / --ang-speed only (no jumps, however fast you drag)
  * Ctrl+C holds the current pose, the logger (--log) works as usual

  ./fr3 run 09_rviz_goal.py
  ./fr3 run 09_rviz_goal.py --tilt --max-tilt 10
  ./fr3 run real 09_rviz_goal.py --lin-speed 0.03 --max-tilt 8

In RViz: ./fr3 up <mode> --rviz already shows it (config/asu_fr3.rviz has the InteractiveMarkers display on /lab_goal, fixed frame base).
With another RViz profile: Add -> By topic -> /lab_goal/update -> InteractiveMarkers.
Do NOT run another script at the same time: CRISP ignores a target topic with more than one publisher.
"""
import argparse
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

import lab_common as L
import lab_config as C

NAME = "goal"
DT = 1.0 / C.PUBLISH_HZ
HARD_TILT_CAP = 30.0   # deg, refuse a larger --max-tilt


def tilt_deg(rot: Rotation) -> float:
    z = rot.apply([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(-z[2], -1.0, 1.0))))


def yaw_deg_of(rot: Rotation) -> float:
    """Rotation about vertical of `rot` relative to the DOWN pose (inverse of L.make_pose, exact when level)."""
    return float(np.degrees((rot * C.DOWN.inv()).as_euler("zyx")[0]))


def limit_tilt(rot: Rotation, max_deg: float):
    """Return (rot', changed): the tool z axis is brought to at most max_deg from straight down, same azimuth."""
    z = rot.apply([0.0, 0.0, 1.0])
    th = float(np.arccos(np.clip(-z[2], -1.0, 1.0)))
    if np.degrees(th) <= max_deg:
        return rot, False
    h = z[:2]
    n = float(np.linalg.norm(h))
    u = h / n if n > 1e-9 else np.array([1.0, 0.0])
    tc = np.radians(max_deg)
    zc = np.array([np.sin(tc) * u[0], np.sin(tc) * u[1], -np.cos(tc)])
    axis = np.cross(z, zc)
    s = float(np.linalg.norm(axis))
    ang = float(np.arctan2(s, float(np.dot(z, zc))))
    return Rotation.from_rotvec(axis / s * ang) * rot, True


def marker_quat(tool_rot: Rotation):
    return Rotation.as_quat(tool_rot * C.DOWN.inv())      # marker orientation = tool orientation relative to DOWN


def build_marker(pos, tool_rot, tilt_rings):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

    im = InteractiveMarker()
    im.header.frame_id = C.BASE_FRAME
    im.name = NAME
    im.description = "hand goal (right-click: menu)"
    im.scale = 0.15
    im.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
    q = marker_quat(tool_rot)
    im.pose.orientation.x, im.pose.orientation.y, im.pose.orientation.z, im.pose.orientation.w = map(float, q)

    ball = InteractiveMarkerControl()
    ball.always_visible = True
    ball.interaction_mode = InteractiveMarkerControl.NONE
    m = Marker()
    m.type = Marker.SPHERE
    m.scale.x = m.scale.y = m.scale.z = 0.03
    m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 0.9
    ball.markers.append(m)
    im.controls.append(ball)

    def control(name, mode, w, x, y, z):
        c = InteractiveMarkerControl()
        c.name, c.interaction_mode = name, mode
        c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = float(w), float(x), float(y), float(z)
        c.orientation_mode = InteractiveMarkerControl.FIXED     # axes stay aligned with the base frame
        im.controls.append(c)

    control("move_x", InteractiveMarkerControl.MOVE_AXIS, 1, 1, 0, 0)
    control("move_y", InteractiveMarkerControl.MOVE_AXIS, 1, 0, 0, 1)
    control("move_z", InteractiveMarkerControl.MOVE_AXIS, 1, 0, 1, 0)
    control("move_xy", InteractiveMarkerControl.MOVE_PLANE, 1, 0, 1, 0)
    control("rotate_z", InteractiveMarkerControl.ROTATE_AXIS, 1, 0, 1, 0)   # yaw, about the vertical axis
    if tilt_rings:
        control("tilt_x", InteractiveMarkerControl.ROTATE_AXIS, 1, 1, 0, 0)
        control("tilt_y", InteractiveMarkerControl.ROTATE_AXIS, 1, 0, 0, 1)
    return im


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to run (0 = until Ctrl+C)")
    ap.add_argument("--ns", default="lab_goal", help="interactive-marker namespace (topics /<ns>/update, /<ns>/feedback)")
    ap.add_argument("--tilt", action="store_true", help="start with the two tilt rings shown")
    ap.add_argument("--max-tilt", type=float, default=15.0, help="largest allowed tilt from straight down [deg] (default 15)")
    L.add_common_args(ap)
    a = ap.parse_args()
    if a.lin_speed > C.LIM["max_lin_speed"]:
        raise SystemExit(f"--lin-speed {a.lin_speed} above the cap {C.LIM['max_lin_speed']} m/s (lab_config.py)")
    if not 0.0 <= a.max_tilt <= HARD_TILT_CAP:
        raise SystemExit(f"--max-tilt must be between 0 and {HARD_TILT_CAP} degrees")

    with L.session(a, "09_rviz_goal.py") as (r, lg):
        from interactive_markers import InteractiveMarkerServer, MenuHandler
        from visualization_msgs.msg import InteractiveMarkerFeedback

        tgt = r.target_pose
        lock = threading.Lock()
        goal = {"pos": np.array(tgt.position, dtype=float), "rot": tgt.orientation}
        flags = {"rebuild": False, "tilt_rings": bool(a.tilt)}
        server = InteractiveMarkerServer(r.node, a.ns)
        menu = MenuHandler()

        def on_feedback(fb):
            if fb.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
                return
            p = np.array([fb.pose.position.x, fb.pose.position.y, fb.pose.position.z])
            q = [fb.pose.orientation.x, fb.pose.orientation.y, fb.pose.orientation.z, fb.pose.orientation.w]
            tool = Rotation.from_quat(q) * C.DOWN
            tool, tilted = limit_tilt(tool, a.max_tilt)
            clamped = np.clip(p, C.BOUNDS_MIN, C.BOUNDS_MAX)
            with lock:
                goal["pos"], goal["rot"] = clamped, tool
            if tilted or not np.allclose(clamped, p):             # snap the marker back inside the limits
                pose = fb.pose
                pose.position.x, pose.position.y, pose.position.z = map(float, clamped)
                qq = marker_quat(tool)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, qq)
                server.setPose(NAME, pose)
                server.applyChanges()

        def on_toggle_tilt(fb):
            flags["tilt_rings"] = not flags["tilt_rings"]
            flags["rebuild"] = True

        def on_level(fb):
            with lock:
                goal["rot"] = L.make_pose(goal["pos"], yaw_deg_of(goal["rot"])).orientation
            flags["rebuild"] = True

        h_tilt = menu.insert("Tilt rings", callback=on_toggle_tilt)
        menu.insert("Level hand (keep yaw)", callback=on_level)

        def publish_marker():
            with lock:
                pos, rot = goal["pos"].copy(), goal["rot"]
            server.erase(NAME)
            server.applyChanges()
            server.insert(build_marker(pos, rot, flags["tilt_rings"]), feedback_callback=on_feedback)
            server.applyChanges()
            menu.setCheckState(h_tilt, MenuHandler.CHECKED if flags["tilt_rings"] else MenuHandler.UNCHECKED)
            menu.apply(server, NAME)
            server.applyChanges()

        publish_marker()
        print(f"\nmarker '{NAME}' published on /{a.ns}/update  (frame {C.BASE_FRAME})")
        print("RViz: Add -> By topic -> InteractiveMarkers (fixed frame: base). Drag the marker; right-click for the menu; Ctrl+C holds the pose.")
        print(f"safety box {C.BOUNDS_MIN} .. {C.BOUNDS_MAX} m, speed {a.lin_speed} m/s, {a.ang_speed} rad/s, max tilt {a.max_tilt} deg\n")
        if lg:
            lg.mark("drag")

        t0 = time.perf_counter()
        last_print = 0.0
        k = 0
        while a.duration <= 0 or time.perf_counter() - t0 < a.duration:
            k += 1
            if flags["rebuild"]:
                flags["rebuild"] = False
                publish_marker()
            with lock:
                gpos, grot = goal["pos"].copy(), goal["rot"]
            cur = r.target_pose
            d = gpos - np.array(cur.position)
            dist = float(np.linalg.norm(d))
            step = min(dist, a.lin_speed * DT)
            npos = np.array(cur.position) + (d / dist * step if dist > 1e-9 else 0.0)
            delta = cur.orientation.inv() * grot
            ang = float(delta.magnitude())
            frac = 1.0 if ang <= a.ang_speed * DT else a.ang_speed * DT / ang
            nrot = cur.orientation * Rotation.from_rotvec(delta.as_rotvec() * frac)
            L.check_bounds(npos)
            r.set_target(pose=L.Pose(npos, nrot))
            now = time.perf_counter() - t0
            if now - last_print >= 2.0:
                last_print = now
                meas = r.end_effector_pose
                print(f"goal {np.round(gpos, 3)} tilt {tilt_deg(grot):4.1f} yaw {yaw_deg_of(grot):6.1f} deg | to goal {dist*1000:6.1f} mm | "
                      f"hand error {np.linalg.norm(meas.position - npos)*1000:5.2f} mm, tilt {tilt_deg(meas.orientation):4.1f} deg")
            time.sleep(max(0.0, t0 + k * DT - time.perf_counter()))
        server.shutdown()


if __name__ == "__main__":
    main()
