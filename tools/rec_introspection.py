"""rec_introspection.py  --  runs INSIDE the stack container (see bin/rec.sh, used by ./fr3 rec).

Records /controller_manager/introspection_data (names + values, 1 kHz) to a CSV:
  state_interface.*   joint position / velocity / effort (real arm: measured torque)
  command_interface.* effort actually sent to the arm (+ .is_limited flag)
  cartesian_impedance_controller.*  tau_desired / tau_task / error / targets / nullspace refs
  *.failed_triggers   missed controller cycles
Needs enable_introspection: true in the cartesian controller block for the cartesian_impedance_controller.* columns; the rest is always recorded.

Column names in the CSV (N = joint number 1 to 7):
  t                      seconds since the first sample          stamp   absolute ROS time of the sample
  qN, dqN                measured joint position [rad] and speed [rad/s]
  tau_measN              REPORTED torque [N m] (real arm: Franka's measured joint torque, gravity included)
  tau_cmdN               COMMANDED torque [N m]: what the controller sent to the hardware
  cmd_limitedN           1 when that command was clipped to a limit
  cycles_<controller>    control cycles run, missed_<controller>  cycles missed (failed triggers)
  rec_*                  statistics of the recording itself (dropped messages)
  anything else          kept under its original name (the cartesian_impedance_controller.* columns)

The file is written WHILE recording (one row at a time, flushed every 2 s): a long recording needs no memory, and if the process is killed
the rows written so far are in the file. Ctrl+C stops early and finishes the file (the vr_* columns are dropped at the end if no headset sent data).

usage (inside the container):  python3 - OUTPUT.csv SECONDS    (script text on stdin; a bare NAME writes /tmp/NAME.csv)
"""
import csv
import os
import sys
import time

import rclpy
from lab_columns import PoseTap, friendly       # shared with the --log logger (scripts/lab_columns.py; ./fr3 puts scripts/ on PYTHONPATH)
from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

target, seconds = sys.argv[1], float(sys.argv[2])
out = target if "/" in target else f"/tmp/{target}.csv"

rclpy.init()
node = rclpy.create_node("lab_rec_introspection")
pose = PoseTap(node)      # TCP, target and VR hand pose, sampled and held at every row
names = {}                # names_version -> controller-manager column names
pending = []              # rows that arrived before their names did
st = dict(file=None, writer=None, ver=None, ncols=0, n=0, t_first=None, last=None, skipped=0, last_pv=None)
vr_idx = [i for i, c in enumerate(pose.columns) if c.startswith("vr_")]
vr_seen = set()


def on_names(m):
    names[m.names_version] = list(m.names)


def write_row(stamp, values, pv):
    if pv is not st["last_pv"]:                       # a new pose snapshot: remember which vr columns ever had data
        st["last_pv"] = pv
        for i in vr_idx:
            if pv[i] == pv[i]:
                vr_seen.add(i)
    st["writer"].writerow([f"{stamp - st['t_first']:.4f}", f"{stamp:.6f}"] + [f"{v:.6g}" for v in values] + [f"{v:.7g}" for v in pv])
    st["n"] += 1
    st["last"] = stamp
    if st["n"] % 2000 == 0:
        st["file"].flush()


def open_file(version, first_stamp):
    cols = names[version]
    st["file"] = open(out, "w", newline="")
    st["writer"] = csv.writer(st["file"])
    st["ver"], st["ncols"], st["t_first"] = version, len(cols), first_stamp
    st["writer"].writerow(["t", "stamp"] + [friendly(c) for c in cols] + pose.columns)


def on_values(m):
    stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    pv = pose.snapshot()
    if st["writer"] is None:
        if m.names_version not in names:              # names not here yet: keep a few rows
            if len(pending) < 5000:
                pending.append((stamp, m.names_version, list(m.values), pv))
            return
        queued = [r for r in pending if r[1] == m.names_version and len(r[2]) == len(names[m.names_version])]
        open_file(m.names_version, queued[0][0] if queued else stamp)
        for r in queued:
            write_row(r[0], r[2], r[3])
        pending.clear()
    if m.names_version != st["ver"] or len(m.values) != st["ncols"]:
        st["skipped"] += 1                            # the controller-manager column set changed: this file keeps the first set
        return
    write_row(stamp, m.values, pv)


qos_n = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
qos_v = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(StatisticsNames, "/controller_manager/introspection_data/names", on_names, qos_n)
node.create_subscription(StatisticsNames, "/controller_manager/introspection_data/names", on_names, 10)
node.create_subscription(StatisticsValues, "/controller_manager/introspection_data/values", on_values, qos_v)

t0 = time.time()
try:
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
except KeyboardInterrupt:        # Ctrl+C: stop early and finish the file
    print(f"stopped after {time.time() - t0:.1f} s")

if st["file"] is None:
    print("no introspection data received (is a controller active? are the names published?)")
    sys.exit(1)
st["file"].close()
dead = [i for i in vr_idx if i not in vr_seen]            # vr_* columns appear only if a headset sent data
if dead:
    first_vr = 2 + st["ncols"]
    drop = {first_vr + i for i in dead}
    with open(out) as fin, open(out + ".tmp", "w", newline="") as fout:
        w = csv.writer(fout)
        for row in csv.reader(fin):
            w.writerow([v for j, v in enumerate(row) if j not in drop])
    os.replace(out + ".tmp", out)
n = st["n"]
dt = (st["last"] - st["t_first"]) / max(1, n - 1) * 1000
print(f"recorded {n} samples, mean period {dt:.3f} ms, {2 + st['ncols'] + len(pose.columns) - len(dead)} columns -> {out}"
      + (f"  ({st['skipped']} rows skipped: the column set changed)" if st["skipped"] else ""))
