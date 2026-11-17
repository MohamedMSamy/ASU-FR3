#!/usr/bin/env python3
"""13_sine_sweep.py  --  frequency response of ONE joint, to identify its effective inertia (link + armature) and damping.

Uses the joint_impedance_controller (a spring-damper per joint, K_j = nullspace.stiffness x weight, D_j = nullspace.damping x weight).
The script moves ONE joint's target on a small sine wave around the pose it starts in, at several frequencies, and records the target
and the measured joint position, velocity and torque. 14_sweep_fit.py then turns the gain and phase at each frequency into
    I_eff (effective inertia), b (viscous damping), the loop delay,   and armature = I_eff - link inertia from the URDF.
Run the SAME command on the real arm and on the sim, and tune the sim's damping/armature until the two fits agree.

  ./fr3 run real 13_sine_sweep.py --joint 4 --log sw_j4_k300_15
  ./fr3 run 13_sine_sweep.py --joint 4 --log sw_j4_k300_15          # the sim, same name: compare afterwards
  ./fr3 run fake 13_sine_sweep.py --joint 2 --dry-run               # print the plan only, no ROS

Safety (all caps are per frequency; the amplitude is reduced to respect them):
  * peak spring torque K_j * A <= --max-torque (default 3 N m). The controller clips the nullspace torque at 5 N m, a larger one distorts the sine.
  * peak joint speed A * 2 pi f <= --max-speed (default 0.4 rad/s).
  * the target starts with a one-cycle ramp-in, and returns to the start pose at the end. Ctrl+C holds the pose.
Start from home (08_home.py); user stop in hand on the real arm; nothing near the arm (the joint moves a few hundredths of a rad).
Notes: crisp_py republishes the target at 50 Hz, so the target is a staircase of 20 ms: that is a delay of about 10 ms, the same on the real
arm and the sim, and the fit estimates it. The joint-state topic and the controller run at 1 kHz; the script samples at --rate (default 100 Hz).
Output: data/logs/<log>_<mode>_<time>_sweep.csv (f, t, target, q, dq, tau) and a .json with the gains, the start pose and the plan.
"""
import argparse
import csv
import json
import time

import numpy as np

import lab_common as L
import lab_config as C

# nullspace.stiffness (5.0) x weights of the joint_impedance_controller block of controllers*.yaml; used for the plan and as a fallback
DEFAULT_K = {1: 750.0, 2: 750.0, 3: 750.0, 4: 750.0, 5: 250.0, 6: 150.0, 7: 50.0}


def plan(freqs, k_j, amp, max_torque, max_speed, cycles):
    rows = []
    for f in freqs:
        w = 2 * np.pi * f
        a = min(amp, max_torque / k_j, max_speed / w)
        dur = max(cycles / f, 4.0)
        rows.append((f, a, k_j * a, a * w, dur))
    return rows


def sine_fit(t, x, f):
    """Complex amplitude C of x(t) ~ Re{C e^{j 2 pi f t}} + offset (least squares)."""
    w = 2 * np.pi * f
    M = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    (a, b, _), *_ = np.linalg.lstsq(M, x, rcond=None)
    return b - 1j * a


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", type=int, choices=range(1, 8), required=True)
    ap.add_argument("--freqs", nargs="+", type=float, default=[0.2, 0.4, 0.8, 1.5, 2.5, 4.0, 6.0, 9.0], help="Hz")
    ap.add_argument("--amplitude", type=float, default=0.02, help="largest amplitude [rad] (reduced per frequency by the caps below)")
    ap.add_argument("--max-torque", type=float, default=3.0, help="peak spring torque K*A [N m]; the controller clips at 5")
    ap.add_argument("--max-speed", type=float, default=0.4, help="peak joint speed [rad/s]")
    ap.add_argument("--cycles", type=float, default=6.0, help="cycles per frequency (at least 4 s each)")
    ap.add_argument("--rate", type=float, default=100.0, help="sampling and target update rate [Hz]")
    ap.add_argument("--k-joint", type=float, default=None, help="joint stiffness K_j [N m/rad] for the plan (default: read from the controller, else the table)")
    ap.add_argument("--dry-run", action="store_true", help="print the plan and stop; no ROS, no robot")
    L.add_common_args(ap, controller_switch=False)
    a = ap.parse_args()
    j = a.joint - 1
    if min(a.freqs) <= 0 or max(a.freqs) > 12.0:
        raise SystemExit("--freqs must be between 0 and 12 Hz (the target is updated at 50 Hz by crisp_py)")
    if a.amplitude > 0.05:
        raise SystemExit("--amplitude above 0.05 rad is refused")

    k0 = a.k_joint or DEFAULT_K[a.joint]
    rows = plan(a.freqs, k0, a.amplitude, a.max_torque, a.max_speed, a.cycles)
    total = sum(r[4] for r in rows) + 4.0

    def show(rows, k):
        print(f"joint {a.joint}: K_j = {k:g} N m/rad;  {len(rows)} frequencies, about {sum(r[4] for r in rows) + 4:.0f} s")
        print(f"{'f [Hz]':>8} {'amplitude [rad]':>16} {'peak torque [N m]':>18} {'peak speed [rad/s]':>19} {'time [s]':>9}")
        for f, amp, tq, sp, dur in rows:
            print(f"{f:8.2f} {amp:16.4f} {tq:18.2f} {sp:19.3f} {dur:9.1f}")

    if a.dry_run:
        show(rows, k0)
        print("dry run: nothing was sent to a robot.")
        return

    with L.session(a, "13_sine_sweep.py", controller=C.JOINT_CONTROLLER) as (r, lg):
        name_j = C.JOINT_NAMES[j]
        ks = ds = None
        try:
            vals = r.joint_controller_parameters_client.get_parameters(
                ["nullspace.stiffness", "nullspace.damping", f"nullspace.weights.{name_j}.value"])
            if vals[0] is not None and vals[2] is not None:
                ks = float(vals[0]) * float(vals[2])
                ds = (float(vals[1]) * float(vals[2])) if (vals[1] is not None and float(vals[1]) > 0) else 2.0 * np.sqrt(ks)
        except Exception as exc:  # noqa: BLE001
            print(f"(could not read the joint gains: {exc})")
        if a.k_joint:
            ks = a.k_joint
        if ks is None:
            ks = DEFAULT_K[a.joint]
            print(f"using the table value K_j = {ks:g} (the controller's parameters were not readable)")
        if ds is None:
            ds = 2.0 * np.sqrt(ks)
            print(f"using D_j = 2 sqrt(K_j) = {ds:.2f} (not readable)")
        rows = plan(a.freqs, ks, a.amplitude, a.max_torque, a.max_speed, a.cycles)
        show(rows, ks)
        print(f"D_j = {ds:g} N m s/rad")
        time.sleep(1.0)
        q0 = np.asarray(r.joint_values, dtype=float).copy()
        print("start pose [rad]:", np.round(q0, 3))
        dt = 1.0 / a.rate
        data = []
        summary = []
        print(f"\n{'f [Hz]':>8} {'gain':>8} {'gain [dB]':>10} {'phase [deg]':>12} {'peak tau [N m]':>15}")
        for f, amp, _, _, dur in rows:
            n = int(round(dur * a.rate))
            t0 = time.perf_counter()
            seg = []
            for k in range(1, n + 1):
                t = k * dt
                env = float(L._smooth(min(t * f, 1.0)))            # one-cycle ramp-in
                qt = q0.copy()
                qt[j] = q0[j] + amp * env * np.sin(2 * np.pi * f * t)
                r.set_target_joint(qt)
                L._sleep_until(t0 + t)
                eff = getattr(r, "_lab_effort", None)
                seg.append((f, t, qt[j], float(r.joint_values[j]), float(r.joint_velocities[j]),
                            float(eff[j]) if eff is not None and np.isfinite(eff[j]) else float("nan")))
            data.extend(seg)
            s = np.array(seg)
            m = s[:, 1] >= max(1.0 / f, 1.0)                        # drop the ramp-in
            ct, cq = sine_fit(s[m, 1], s[m, 2], f), sine_fit(s[m, 1], s[m, 3], f)
            g = cq / ct
            summary.append((f, abs(g), float(np.degrees(np.angle(g)))))
            print(f"{f:8.2f} {abs(g):8.3f} {20 * np.log10(abs(g)):10.2f} {np.degrees(np.angle(g)):12.1f} {np.nanmax(np.abs(s[m, 5] - np.nanmean(s[m, 5]))):15.2f}")
        r.set_target_joint(q0)
        time.sleep(2.0)
        name = a.log or f"sweep_j{a.joint}"
        C.LOG_DIR.mkdir(parents=True, exist_ok=True)
        base = C.LOG_DIR / f"{name}_{C.MODE}_{time.strftime('%Y%m%d_%H%M%S')}"
        with open(f"{base}_sweep.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["f_hz", "t", "q_target", "q", "dq", "tau"])
            w.writerows(data)
        with open(f"{base}_sweep.json", "w") as fh:
            json.dump({"script": "13_sine_sweep.py", "mode": C.MODE, "joint": a.joint, "K_j": ks, "D_j": ds, "q0": q0.tolist(),
                       "rate_hz": a.rate, "plan": [dict(f=p[0], amplitude=p[1]) for p in rows],
                       "controller_params": L.controller_params(r, which="joint")}, fh, indent=2, default=str)
        print(f"\nsweep written: {base}_sweep.csv  (fit it with 14_sweep_fit.py)")


if __name__ == "__main__":
    main()
