"""rec_introspection.py  --  runs INSIDE the stack container (see bin/rec.sh, used by ./fr3 rec).

Records /controller_manager/introspection_data (names + values, 1 kHz) to a CSV:
  state_interface.*   joint position / velocity / effort (real arm: measured torque)
  command_interface.* effort actually sent to the arm (+ .is_limited flag)
  cartesian_impedance_controller.*  tau_desired / tau_task / error / targets / nullspace refs
  *.failed_triggers   missed controller cycles
Needs enable_introspection: true in the cartesian controller block.

usage (inside the container):  python3 - NAME SECONDS    (script text on stdin)
"""
import csv
import sys
import time

import rclpy
from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

name, seconds = sys.argv[1], float(sys.argv[2])
rclpy.init()
node = rclpy.create_node("lab_rec_introspection")
names = {}      # version -> list of names
rows = []       # (stamp, version, values)


def on_names(m):
    names[m.names_version] = list(m.names)


def on_values(m):
    rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.names_version, list(m.values)))


qos_n = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
qos_v = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(StatisticsNames, "/controller_manager/introspection_data/names", on_names, qos_n)
node.create_subscription(StatisticsNames, "/controller_manager/introspection_data/names", on_names, 10)
node.create_subscription(StatisticsValues, "/controller_manager/introspection_data/values", on_values, qos_v)

t0 = time.time()
while time.time() - t0 < seconds:
    rclpy.spin_once(node, timeout_sec=0.05)

if not rows:
    print("no introspection data received (is enable_introspection true and a controller active?)")
    sys.exit(1)
ver = rows[-1][1]
if ver not in names:
    print(f"names for version {ver} not received; try a longer duration")
    sys.exit(1)
cols = names[ver]
good = [r for r in rows if r[1] == ver and len(r[2]) == len(cols)]
out = f"/tmp/{name}.csv"
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["stamp"] + cols)
    for st, _, vals in good:
        w.writerow([f"{st:.6f}"] + [f"{v:.6g}" for v in vals])
dt = (good[-1][0] - good[0][0]) / max(1, len(good) - 1) * 1000
print(f"recorded {len(good)} samples, mean period {dt:.3f} ms, {len(cols)} columns -> {out}")
