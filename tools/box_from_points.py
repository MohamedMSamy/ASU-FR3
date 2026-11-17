"""Turn the points recorded with 07_record_waypoints.py into the LIMITS["real"] lines for scripts/lab_config.py.

  python3 tools/box_from_points.py box_points.csv                 # 2 cm inside the extreme points on every axis
  python3 tools/box_from_points.py box_points.csv --margin 0.03 --z-low-margin 0.005

bounds_min = lowest recorded point + margin, bounds_max = highest recorded point - margin.
The lowest z is normally the fingertip touching the table, so --z-low-margin is the clearance you keep above it.
It only prints; you paste the two lines into lab_config.py yourself. Plain Python, no ROS needed.
"""

import argparse


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--margin", type=float, default=0.02, help="m kept inside the extreme points (default 0.02)")
    ap.add_argument("--z-low-margin", type=float, default=None, help="m kept above the lowest z (default: --margin)")
    a = ap.parse_args()
    zm = a.margin if a.z_low_margin is None else a.z_low_margin

    pts = []
    for line in open(a.csv):
        line = line.strip()
        if line and not line.startswith("#"):
            pts.append([float(v) for v in line.split(",")[:3]])
    if len(pts) < 2:
        raise SystemExit("need at least 2 points")

    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    print(f"{len(pts)} points   extreme min {[round(v, 3) for v in lo]}   extreme max {[round(v, 3) for v in hi]}")
    print(f"extent x {hi[0] - lo[0]:.3f}  y {hi[1] - lo[1]:.3f}  z {hi[2] - lo[2]:.3f} m")
    bmin = [lo[0] + a.margin, lo[1] + a.margin, lo[2] + zm]
    bmax = [hi[0] - a.margin, hi[1] - a.margin, hi[2] - a.margin]
    if any(b <= s for s, b in zip(bmin, bmax)):
        raise SystemExit("margin is larger than the box: record more spread-out points or use a smaller --margin")
    print(f"\npaste into LIMITS[\"real\"] in scripts/lab_config.py (margin {a.margin} m, z-low {zm} m):")
    print(f"        bounds_min=np.array([{bmin[0]:.3f}, {bmin[1]:.3f}, {bmin[2]:.3f}]),")
    print(f"        bounds_max=np.array([{bmax[0]:.3f}, {bmax[1]:.3f}, {bmax[2]:.3f}]),")


if __name__ == "__main__":
    main()
