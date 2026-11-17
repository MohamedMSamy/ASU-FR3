#!/usr/bin/env python3
"""14_sweep_fit.py  --  fit effective inertia, damping and delay to a sine sweep of 13_sine_sweep.py (offline, no ROS).

Model of one joint under the joint_impedance_controller (spring K_j, damper D_j, target q_t, measured q):
    I q'' + b q' + friction = K_j (q_t - q) - D_j q'      =>      G(jw) = q/q_t = K_j / ( K_j - I w^2 + j w (b + D_j) )  x  exp(-j w T)
I = effective inertia of the joint (link + motor armature), b = viscous damping, T = delay (crisp_py's 50 Hz target hold, filters).
Gain and phase at every swept frequency come from a least-squares sine fit; I, b and T from a least-squares fit of the complex gains.
If Pinocchio and the exported URDF are available (./fr3 urdf writes data/urdf/fr3.urdf) it also gives the link inertia M_jj at the pose of the
sweep, so  armature ~ I_eff - M_jj  (the quantity the MuJoCo XML calls armature). Sim and real are fitted separately: tune the XML until they agree.

  ./fr3 run fake 14_sweep_fit.py "/home/ros/data/logs/sw_j4_k300_15_real_*_sweep.csv"
  ./fr3 run fake 14_sweep_fit.py "/home/ros/data/logs/sw_j4_k300_15_real_*_sweep.csv" "/home/ros/data/logs/sw_j4_k300_15_sim_*_sweep.csv"   # side by side

Reading it: the fit is only as good as the sweep. Look at the residual: large values mean friction (stick-slip at low amplitude), saturation of the
nullspace torque (5 N m) or a joint that hit a limit. A friction dead band makes the low-frequency gain fall below 1. The first frequency
(0.2 Hz) mostly shows K_j and friction, the resonance region shows I and b.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares


def sine_fit(t, x, f):
    w = 2 * np.pi * f
    M = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    (a, b, _), *_ = np.linalg.lstsq(M, x, rcond=None)
    return b - 1j * a


def gains(path):
    """[(f, G complex)], K_j, D_j, joint, q0 from a *_sweep.csv and the .json next to it."""
    d = np.genfromtxt(path, delimiter=",", names=True)
    meta = json.load(open(path[: -len("_sweep.csv")] + "_sweep.json"))
    out = []
    for f in sorted(set(d["f_hz"])):
        s = d[d["f_hz"] == f]
        m = s["t"] >= max(1.0 / f, 1.0)
        out.append((f, sine_fit(s["t"][m], s["q"][m], f) / sine_fit(s["t"][m], s["q_target"][m], f)))
    return out, float(meta["K_j"]), float(meta["D_j"]), int(meta["joint"]), np.array(meta["q0"])


def model(f, K, D, I, b, T):
    w = 2 * np.pi * np.asarray(f)
    return K / (K - I * w ** 2 + 1j * w * (b + D)) * np.exp(-1j * w * T)


def fit(g, K, D, I0):
    f = np.array([x[0] for x in g])
    G = np.array([x[1] for x in g])
    if len(f) < 4:
        print(f"  WARNING: only {len(f)} frequencies: I, b and the delay are three unknowns, use at least 4 (the default plan has 8)")

    def res(p):
        e = model(f, K, D, p[0], p[1], p[2]) - G
        return np.concatenate([e.real, e.imag])

    best = None
    for I_try in (I0 * 0.3, I0, I0 * 3.0):       # a few starts: the resonance makes the surface bumpy
        sol = least_squares(res, [I_try, 1.0, 0.01], bounds=([1e-4, 0.0, 0.0], [50.0, 300.0, 0.06]), x_scale=[1.0, 1.0, 0.01])
        if best is None or sol.cost < best.cost:
            best = sol
    I, b, T = best.x
    return I, b, T, float(np.sqrt(2 * best.cost / len(f)))


def link_inertia(urdf, q0, j):
    try:
        import pinocchio as pin
        m = pin.buildModelFromUrdf(urdf)
        m = pin.buildReducedModel(m, [m.getJointId(n) for n in m.names if n.startswith("fr3_finger")], np.zeros(m.nq))
        d = m.createData()
        pin.crba(m, d, np.asarray(q0, dtype=float))
        M = np.triu(d.M) + np.triu(d.M, 1).T
        return float(M[j - 1, j - 1])
    except Exception:  # noqa: BLE001
        return None


def report(path, urdf):
    g, K, D, joint, q0 = gains(path)
    mjj = link_inertia(urdf, q0, joint) if urdf and os.path.exists(urdf) else None
    I0 = mjj if mjj else max(K / (2 * np.pi * 3.0) ** 2, 1e-3)
    I, b, T, rms = fit(g, K, D, I0)
    print(f"\n{os.path.basename(path)}   joint {joint}   K_j {K:g}   D_j {D:g}")
    print(f"{'f [Hz]':>8} {'gain':>8} {'phase[deg]':>11} | {'model gain':>10} {'phase[deg]':>11}")
    for f, G in g:
        Gm = model(f, K, D, I, b, T)
        print(f"{f:8.2f} {abs(G):8.3f} {np.degrees(np.angle(G)):11.1f} | {abs(Gm):10.3f} {np.degrees(np.angle(Gm)):11.1f}")
    zeta = (b + D) / (2 * np.sqrt(K * I))
    wn = np.sqrt(K / I)
    print(f"  fit: I_eff = {I:.4f} kg m^2   b = {b:.3f} N m s/rad   delay = {T * 1000:.1f} ms   (rms error of the complex gain {rms:.3f})")
    print(f"       natural frequency {wn / (2 * np.pi):.2f} Hz   total damping ratio {zeta:.2f}")
    if mjj is not None:
        print(f"       link inertia from the URDF at this pose M_jj = {mjj:.4f} kg m^2   =>   armature ~ {I - mjj:+.4f} kg m^2")
    else:
        print("       (no link inertia: pass --urdf data/urdf/fr3.urdf, made by ./fr3 urdf, and run where Pinocchio is installed: inside the image)")
    return dict(I=I, b=b, T=T, mjj=mjj)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("sweeps", nargs="+", help="*_sweep.csv files (globs are expanded); the newest match of each glob is used")
    ap.add_argument("--urdf", default=os.environ.get("FR3_URDF", "/home/ros/data/urdf/fr3.urdf"))
    a = ap.parse_args()
    for pat in a.sweeps:
        hits = sorted(glob.glob(pat)) or [pat]
        if not os.path.exists(hits[-1]):
            sys.exit(f"{pat}: no such file")
        report(hits[-1], a.urdf)


if __name__ == "__main__":
    main()
