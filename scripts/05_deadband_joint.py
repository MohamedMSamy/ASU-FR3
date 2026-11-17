#!/usr/bin/env python3
"""05_deadband_joint.py  --  the same staircase test, but on ONE joint (joint-space friction).

Uses the joint_impedance_controller (a stiff spring on each joint towards /target_joint,
configured in controllers.yaml via nullspace.stiffness x weights).  The dead band you find
times the joint's stiffness K_j is the torque the joint needed before it moved, i.e. the
EFFECTIVE breakaway friction of that joint (what really matters for control; on the real
arm it also includes the inner torque loop's behaviour).

NOTE: the joint_impedance_controller block in controllers.yaml has use_friction: true,
which already adds friction compensation.  For the RAW friction set use_friction: false
in that block (restart), or leave it to see what the compensated joint does.

  ./fr3 run 05_deadband_joint.py --joint 2 --amplitude 0.01 --step 0.0005 --log dbj2
  ./fr3 run real 05_deadband_joint.py --joint 4 --amplitude 0.01 --step 0.0005 --log dbj4

Units: radians.  Default amplitude 0.01 rad (0.57 deg), well below any visible motion.
"""
import argparse
import csv
import time

import numpy as np

import lab_common as L
import lab_config as C
from lab_analysis import analyse_staircase, staircase_sequence


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", type=int, choices=range(1, 8), required=True)
    ap.add_argument("--amplitude", type=float, default=0.01, help="rad")
    ap.add_argument("--step", type=float, default=0.0005, help="rad")
    ap.add_argument("--dwell", type=float, default=1.5)
    ap.add_argument("--allow-large", action="store_true", help="allow amplitude above 0.05 rad")
    L.add_common_args(ap, controller_switch=False)
    a = ap.parse_args()
    if a.amplitude > 0.05 and not a.allow_large:
        raise SystemExit("amplitude above 0.05 rad needs --allow-large")
    j = a.joint - 1
    seq = staircase_sequence(a.amplitude, a.step)
    print(f"joint {a.joint}: {len(seq)} steps, +-{a.amplitude} rad, step {a.step} rad, dwell {a.dwell}s "
          f"= {len(seq) * a.dwell:.0f}s")

    with L.session(a, "05_deadband_joint.py", controller=C.JOINT_CONTROLLER) as (r, lg):
        # Stiffness of this joint = nullspace.stiffness * weight_j (joint_impedance_controller).
        kj = None
        try:
            vals = r.joint_controller_parameters_client.get_parameters(
                ["nullspace.stiffness", f"nullspace.weights.{C.JOINT_NAMES[j]}.value"])
            if all(v is not None for v in vals):
                kj = float(vals[0]) * float(vals[1])
        except Exception as exc:  # noqa: BLE001
            print(f"(could not read joint stiffness: {exc})")
        print(f"joint stiffness K_j = {kj if kj is not None else 'unknown'} Nm/rad")
        time.sleep(1.0)
        q0 = r.joint_values.astype(float)
        cmds, meas, phases, stds = [], [], [], []
        print(f"{'step':>4} {'phase':>5} {'cmd[rad]':>10} {'meas[rad]':>10} {'lag':>9}")
        q_start = float(q0[j])
        for k, (off, phase) in enumerate(seq):
            q = q0.copy()
            q[j] = q0[j] + off
            r.set_target_joint(q)
            if lg:
                lg.mark(f"{phase}_{k}")
            t0 = time.time()
            tail = []
            while time.time() - t0 < a.dwell:
                time.sleep(0.02)
                if time.time() - t0 > a.dwell - 0.5:
                    tail.append(float(r.joint_values[j]))
            m = float(np.mean(tail)) - q_start
            sign = -1.0 if phase == "desc" else 1.0
            cmds.append(off), meas.append(m), phases.append(phase), stds.append(float(np.std(tail)))
            print(f"{k:4d} {phase:>5} {off:10.5f} {m:10.5f} {(off - m) * sign:9.5f}")
        r.set_target_joint(q0)

    noise = max(3 * float(np.median(stds)), 0.02 * a.step)
    res = analyse_staircase(cmds, meas, phases, noise_floor=noise)
    print("\nRESULT")
    print(f"  max lag               : {res['max_lag']:.5f} rad")
    print(f"  steady lag (half-width): {res['steady_lag']:.5f} rad")
    print(f"  hysteresis width      : {res['hysteresis_width']:.5f} rad")
    if kj:
        print(f"  => effective breakaway torque ~ K_j x steady lag = {kj * res['steady_lag']:.3f} Nm "
              f"(worst case {kj * res['max_lag']:.3f} Nm)")
        print("     compare with the URDF value 0.2 Nm (nominal) and the MuJoCo frictionloss in the model")

    name = a.log or f"dbj{a.joint}"
    out = C.LOG_DIR / f"{name}_{C.MODE}_{time.strftime('%Y%m%d_%H%M%S')}_steps.csv"
    C.LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "phase", "cmd", "meas", "lag", "unit", "axis"])
        for k, (c, m, ph, lag) in enumerate(zip(cmds, meas, phases, res["lag_per_step"])):
            w.writerow([k, ph, c, m, lag, "rad", f"joint{a.joint}"])
    print(f"  steps written: {out}")


if __name__ == "__main__":
    main()
