#!/usr/bin/env python3
"""03_geometry.py  --  make the end-effector trace a shape at a constant speed.

Shapes : line, square, rectangle, triangle, circle, ellipse, figure8, spiral
Planes : xy (horizontal, default), xz, yz (vertical planes, hand stays pointing down)

Size is given as --width / --height in metres (for a circle, --width is the DIAMETER).
Speed follows a trapezoid: accelerate at --accel, cruise at --speed, brake to rest, so
there are no jumps in the target.  Closed shapes are looped --loops times without stopping.

  # 1. ALWAYS preview first (no ROS, works on a laptop): prints length, time, accelerations
  #    and writes the planned path to data/logs/ (and a PNG if matplotlib is installed)
  ./fr3 run fake 03_geometry.py --shape circle --center 0.45 0 0.45 --width 0.10 --speed 0.05 --dry-run

  # 2. run it
  ./fr3 run 03_geometry.py --shape circle --center 0.45 0 0.45 --width 0.10 --speed 0.05 --loops 3 --log circle
  ./fr3 run real 03_geometry.py --shape square --center 0.45 0 0.45 --width 0.06 --speed 0.02 --log square

  # vertical circle in the xz plane, hand rotated 30 degrees about vertical:
  ./fr3 run 03_geometry.py --shape circle --plane xz --center 0.45 0 0.5 --width 0.12 --yaw-deg 30

At the end it prints the tracking error (measured pose vs the target you streamed),
which is the number to compare between sim and real.
"""
import argparse
import time

import numpy as np

import lab_config as C
from lab_geometry import PLANES, SHAPES, TrapezoidProfile, make_path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shape", choices=SHAPES, required=True)
    ap.add_argument("--plane", choices=PLANES, default="xy")
    ap.add_argument("--center", nargs=3, type=float, default=list(C.READY_POS), metavar=("X", "Y", "Z"))
    ap.add_argument("--width", type=float, default=0.10, help="shape width (circle: diameter) [m]")
    ap.add_argument("--height", type=float, default=None, help="shape height [m] (default = width; figure8: width/2)")
    ap.add_argument("--turns", type=float, default=2.0, help="spiral turns")
    ap.add_argument("--r-inner", type=float, default=0.01, help="spiral inner radius [m] (0 = singular centre)")
    ap.add_argument("--speed", type=float, default=0.05, help="cruise speed along the path [m/s]")
    ap.add_argument("--accel", type=float, default=0.10, help="speed-up/brake acceleration [m/s^2]")
    ap.add_argument("--loops", type=int, default=1, help="traversals of the shape")
    ap.add_argument("--yaw-deg", type=float, default=0.0)
    ap.add_argument("--dry-run", action="store_true", help="preview only; no ROS, no robot")
    import lab_common as L  # after parsing so --help works anywhere
    L.add_common_args(ap)
    a = ap.parse_args()

    # figure8 looks right at half height (a true lemniscate of Gerono); everything else defaults to square-ish
    h = a.height if a.height is not None else (a.width / 2 if a.shape == "figure8" else a.width)
    path = make_path(a.shape, a.plane, a.center, a.width, h, a.turns, a.r_inner)
    if a.speed > C.LIM["max_lin_speed"]:
        raise SystemExit(f"--speed {a.speed} above the cap {C.LIM['max_lin_speed']} m/s for mode {C.MODE} (lab_config.py)")

    total = path.length * a.loops
    prof = TrapezoidProfile(total, a.speed, a.accel)
    kappa, corner = path.curvature_and_corners()
    a_centripetal = prof.v_max ** 2 * kappa
    corner_jump = prof.v_max * 2 * np.sin(corner / 2) if corner > 0 else 0.0
    print(f"shape {a.shape} in plane {a.plane}, centre {np.round(a.center, 3)}, "
          f"{a.width:.3f} x {h:.3f} m, {'closed' if path.closed else 'open'}")
    print(f"  path length {path.length:.3f} m per traversal, {a.loops} traversal(s) = {total:.3f} m")
    print(f"  duration {prof.duration:.1f}s, peak speed {prof.v_max:.3f} m/s, tangential accel {a.accel} m/s^2")
    print(f"  centripetal accel at peak speed: {a_centripetal:.2f} m/s^2 (limit in lab_config: {C.LIM['max_accel']})")
    if corner > 0:
        print(f"  sharp corner of {np.degrees(corner):.0f} deg: velocity jumps by {corner_jump:.3f} m/s there "
              f"(limit in lab_config: {C.LIM['max_corner_jump']})")
    problems = []
    if max(a_centripetal, a.accel) > C.LIM["max_accel"]:
        problems.append("acceleration above max_accel: lower --speed, enlarge the shape, or (spiral) raise --r-inner")
    if corner_jump > C.LIM["max_corner_jump"]:
        problems.append("corner velocity jump above max_corner_jump: lower --speed")
    for pr in problems:
        print("  !!", pr)
    if problems and not a.dry_run:
        raise SystemExit("refusing to run (limits are in lab_config.py LIMITS)")
    lo, hi = path.pts.min(axis=0), path.pts.max(axis=0)
    print(f"  path box: {np.round(lo, 3)} .. {np.round(hi, 3)}")
    for p in (lo, hi):
        try:
            _check(p)
        except ValueError as exc:
            print("  !!", exc)
            if not a.dry_run:
                raise SystemExit(1)

    planned = np.array([path.point(prof.dist(t)) for t in np.arange(0, prof.duration, 1.0 / C.PUBLISH_HZ)])
    if a.dry_run:
        C.LOG_DIR.mkdir(parents=True, exist_ok=True)
        out = C.LOG_DIR / f"plan_{a.shape}_{a.plane}.csv"
        np.savetxt(out, planned, delimiter=",", header="x,y,z", comments="")
        print(f"  planned path written to {out}")
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(5, 5))
            i, j = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}[a.plane]
            ax.plot(planned[:, i], planned[:, j], "-", lw=1)
            ax.plot(planned[0, i], planned[0, j], "go", label="start")
            ax.set_aspect("equal"), ax.grid(True), ax.legend()
            ax.set_xlabel("xyz"[i] + " [m]"), ax.set_ylabel("xyz"[j] + " [m]")
            png = str(out).replace(".csv", ".png")
            fig.savefig(png, dpi=120, bbox_inches="tight")
            print(f"  preview image: {png}")
        except Exception as exc:  # noqa: BLE001
            print(f"  (no preview image: {exc})")
        print("dry run done: nothing was sent to a robot.")
        return

    goal_start = L.make_pose(path.point(0.0), a.yaw_deg)
    rot = goal_start.orientation

    with L.session(a, "03_geometry.py") as (r, lg):
        print("\nmoving to the start of the path ...")
        L.ramp_to(r, goal_start, a.lin_speed, a.ang_speed, lg, label="to_start")
        time.sleep(1.5)
        print(f"tracing ({prof.duration:.0f}s) ... Ctrl+C holds the pose")
        if lg:
            lg.mark("trace")

        def fn(t):
            return L.Pose(path.point(prof.dist(t)), rot)

        L.stream(r, fn, prof.duration)
        r.set_target(pose=L.Pose(path.point(total), rot))
        time.sleep(2.0)
        if lg:
            lg.mark("end")
            pe = lg.column("pos_err_mm", "trace")
            ae = lg.column("ang_err_deg", "trace")
            if len(pe):
                from lab_analysis import rms
                print("\nTRACKING (measured vs streamed target, during the trace)")
                print(f"  position: RMS {rms(pe):.2f} mm, max {np.nanmax(pe):.2f} mm")
                print(f"  orientation: RMS {rms(ae):.2f} deg, max {np.nanmax(ae):.2f} deg")
        else:
            print("(add --log NAME to record tracking error)")


def _check(p):
    import lab_common as L
    L.check_bounds(p)


if __name__ == "__main__":
    main()
