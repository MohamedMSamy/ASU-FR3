"""Summarise hold logs taken at different gains: is the steady-state error a constant FORCE, or not?

  python3 tools/ksweep_summary.py kswp_p            # every log whose name starts with kswp_p (latest run of each name)
  python3 tools/ksweep_summary.py kswp_p kswp_c     # several prefixes
  python3 tools/ksweep_summary.py holdside --sim    # the sim logs instead of the real ones

For each log it prints the mean over the last 5 s of the hold: error vector (pos - target, mm), K from the
log's json, force-equivalent K*err (N), orientation error and the largest joint speed (is the arm at rest?).
Reading it:  K*err about the same at every K  -> a constant disturbance force (tool mass in Desk, cable, gravity model);
             err about the same at every K    -> not a force problem (stiction, sensing, pose offset);
             err falling as 1/K               -> just needs more stiffness.
Plain Python, reads only the 100 Hz CSV columns it needs. No ROS.
"""

import csv
import glob
import json
import os
import re
import statistics as st
import sys

LOGS = os.environ.get("FR3_LOG_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs"))


def latest_by_name(prefix, mode="real"):
    best = {}
    for f in sorted(glob.glob(os.path.join(LOGS, f"{prefix}*_{mode}_*.csv"))):
        b = os.path.basename(f)
        if b.endswith("_steps.csv") or b.endswith("_intro.csv"):
            continue
        name = re.sub(rf"_{mode}_\d{{8}}_\d{{6}}\.csv$", "", b)
        best[name] = f  # sorted by timestamp in the name, last one wins
    return best


def summarize(name, f):
    rows = [r for r in csv.DictReader(open(f)) if r["tag"] == "hold"]
    if not rows:
        print(f"{name:28s} no 'hold' rows")
        return
    t_end = float(rows[-1]["t"])
    last = [r for r in rows if float(r["t"]) > t_end - 5.0]
    mean = lambda k: st.mean(float(r[k]) for r in last)
    err = [1000.0 * (mean(f"pos_{a}") - mean(f"tgt_pos_{a}")) for a in "xyz"]
    try:
        p = json.load(open(f[:-4] + ".json"))["controller_params"]
        K = [float(p[f"task.k_pos_{a}"]) for a in "xyz"]
    except Exception:  # noqa: BLE001
        K = [float("nan")] * 3
    force = [k * e / 1000.0 for k, e in zip(K, err)]
    dq = max(abs(float(r[f"dq{i}"])) for r in last for i in range(1, 8))
    n = lambda v: sum(x * x for x in v) ** 0.5
    print(
        f"{name:28s} K {K[0]:6.0f}  err mm [{err[0]:6.2f} {err[1]:6.2f} {err[2]:6.2f}] |{n(err):5.2f}|"
        f"  K*err N [{force[0]:5.2f} {force[1]:5.2f} {force[2]:5.2f}] |{n(force):5.2f}|"
        f"  ang {mean('ang_err_deg'):4.2f} deg  max|dq| {dq:.4f}"
    )


def main():
    args = [a for a in sys.argv[1:] if a != "--sim"]
    mode = "sim" if "--sim" in sys.argv else "real"
    for pre in args or ["kswp"]:
        found = latest_by_name(pre, mode)
        if not found:
            print(f"no {mode} logs starting with '{pre}'")
        for name, f in sorted(found.items()):
            summarize(name, f)


if __name__ == "__main__":
    main()
