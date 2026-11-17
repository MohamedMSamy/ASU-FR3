#!/usr/bin/env python3
"""04_deadband_cartesian.py  --  measure the Cartesian dead band / hysteresis (friction signature).

Idea: command a staircase of tiny offsets 0 -> +A -> -A -> 0 along ONE axis, wait at each
step, and record where the hand actually ends up.  With no friction, measured == commanded.
With dry friction the hand lags behind the command by a roughly constant amount (the dead
band) and the up-going and down-going curves do not coincide (hysteresis).  Run the same
test on the sim and on the real arm, then compare the numbers: that is how you tune the
sim's friction to match the robot.

Axes: x y z (millimetres)   rx ry rz (degrees, rotation about the BASE x/y/z axis)

  ./fr3 run 04_deadband_cartesian.py --axis ry --amplitude 3 --step 0.25 --log db_ry
  ./fr3 run real 04_deadband_cartesian.py --axis ry --amplitude 2 --step 0.25 --log db_ry
  ./fr3 run 04_deadband_cartesian.py --axis x  --amplitude 5 --step 0.5  --log db_x

Result CSV: data/logs/<log>_<mode>_<time>_steps.csv  (columns cmd, meas, phase, lag) which
06_compare_logs.py can overlay for sim vs real.
"""
import argparse
import csv
import time

import numpy as np
from scipy.spatial.transform import Rotation

import lab_common as L
import lab_config as C
from lab_analysis import analyse_staircase, staircase_sequence

AXES = {"x": 0, "y": 1, "z": 2, "rx": 0, "ry": 1, "rz": 2}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", choices=list(AXES), default="ry")
    ap.add_argument("--amplitude", type=float, default=None, help="mm or deg (default 5 mm / 3 deg)")
    ap.add_argument("--step", type=float, default=None, help="mm or deg (default 0.5 mm / 0.25 deg)")
    ap.add_argument("--dwell", type=float, default=2.0, help="seconds to wait at each step")
    ap.add_argument("--pos", nargs=3, type=float, default=list(C.READY_POS), metavar=("X", "Y", "Z"))
    ap.add_argument("--allow-large", action="store_true", help="allow amplitude above 10 mm / 8 deg")
    L.add_common_args(ap)
    a = ap.parse_args()

    rot_axis = a.axis.startswith("r")
    unit = "deg" if rot_axis else "mm"
    amp = a.amplitude if a.amplitude is not None else (3.0 if rot_axis else 5.0)
    step = a.step if a.step is not None else (0.25 if rot_axis else 0.5)
    limit = 8.0 if rot_axis else 10.0
    if amp > limit and not a.allow_large:
        raise SystemExit(f"amplitude {amp} {unit} > {limit}; use --allow-large if you really mean it")
    ax = AXES[a.axis]
    base = L.make_pose(a.pos, 0.0)
    L.check_bounds(base.position)
    seq = staircase_sequence(amp, step)
    est = len(seq) * a.dwell
    print(f"{a.axis}: staircase 0 -> +{amp} -> -{amp} -> 0 {unit}, step {step} {unit}, "
          f"{len(seq)} steps x {a.dwell}s = {est:.0f}s")

    def cmd_pose(off):
        if rot_axis:
            vec = np.zeros(3)
            vec[ax] = np.radians(off)
            return L.Pose(base.position.copy(), Rotation.from_rotvec(vec) * base.orientation)
        pos = base.position.copy()
        pos[ax] += off / 1000.0
        return L.Pose(pos, base.orientation)

    def measured_offset():
        cur = r.end_effector_pose
        if rot_axis:
            return float(np.degrees((cur.orientation * base.orientation.inv()).as_rotvec()[ax]))
        return float((cur.position[ax] - base.position[ax]) * 1000.0)

    with L.session(a, "04_deadband_cartesian.py") as (r, lg):
        L.ramp_to(r, base, a.lin_speed, a.ang_speed, lg, label="to_base")
        time.sleep(3.0)
        base_meas = measured_offset()      # where the hand really is at "zero"
        print(f"settled; measured offset at zero command = {base_meas:+.3f} {unit}")
        cmds, meas, phases, stds = [], [], [], []
        print(f"{'step':>4} {'phase':>5} {'cmd[' + unit + ']':>10} {'meas[' + unit + ']':>10} {'lag':>8}")
        for k, (off, phase) in enumerate(seq):
            r.set_target(pose=cmd_pose(off))
            if lg:
                lg.mark(f"{phase}_{k}")
            t0 = time.time()
            tail = []
            while time.time() - t0 < a.dwell:
                time.sleep(0.02)
                if time.time() - t0 > a.dwell - 0.5:
                    tail.append(measured_offset())
            m = float(np.mean(tail)) - base_meas
            sign = -1.0 if phase == "desc" else 1.0
            cmds.append(off), meas.append(m), phases.append(phase), stds.append(float(np.std(tail)))
            print(f"{k:4d} {phase:>5} {off:10.3f} {m:10.3f} {(off - m) * sign:8.3f}")
        r.set_target(pose=base)

    noise = max(3 * float(np.median(stds)), 0.02 * step)
    res = analyse_staircase(cmds, meas, phases, noise_floor=noise)
    print("\nRESULT (same unit as the axis: " + unit + ")")
    print(f"  max lag (worst case behind the command) : {res['max_lag']:.3f}")
    print(f"  steady lag once moving (~ dead-band half-width): {res['steady_lag']:.3f}")
    print(f"  hysteresis width (down-curve minus up-curve)    : {res['hysteresis_width']:.3f}")
    for leg, v in res["breakaway_cmd_travel"].items():
        txt = "never moved" if v is None else f"{v:.3f}"
        print(f"  breakaway, leg {leg:5s}: command had to travel {txt} before the hand moved")

    if lg:
        pass  # the logger CSV is written by the session; also write the steps file
    name = a.log or f"db_{a.axis}"
    out = C.LOG_DIR / f"{name}_{C.MODE}_{time.strftime('%Y%m%d_%H%M%S')}_steps.csv"
    C.LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "phase", "cmd", "meas", "lag", "unit", "axis"])
        for k, (c, m, ph, lag) in enumerate(zip(cmds, meas, phases, res["lag_per_step"])):
            w.writerow([k, ph, c, m, lag, unit, a.axis])
    print(f"  steps written: {out}")


if __name__ == "__main__":
    main()
