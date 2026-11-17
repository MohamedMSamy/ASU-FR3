#!/usr/bin/env bash
# Record the controller's 1 kHz introspection stream (torques, errors, joint state, missed cycles) to a CSV.
# Runs INSIDE the stack container; normally started with ./fr3 rec [mode] NAME SECONDS  (copy of crisp_lab/bin/rec.sh).
# Needs enable_introspection: true in the cartesian block of the controllers file.
#   ./fr3 rec mujoco hold_side 30        # -> data/logs/hold_side_sim_<time>_intro.csv
#   ./fr3 rec real   hold_side 30
# Start it a second or two BEFORE the test, give it a few seconds more than the test lasts.
set -euo pipefail
NAME="${1:?usage: rec.sh NAME SECONDS}"; SECS="${2:?SECONDS}"
MODE="${CRISP_LAB_MODE:-sim}"
STAMP="$(date +%Y%m%d_%H%M%S)"; REMOTE="rec_${NAME}_${MODE}_${STAMP}"
LOGS="${FR3_LOG_DIR:-/home/ros/data/logs}"; mkdir -p "$LOGS"
for d in /ws_override/tools /opt/asu_fr3/tools; do [ -f "$d/rec_introspection.py" ] && { T="$d/rec_introspection.py"; break; }; done
python3 - "$REMOTE" "$SECS" < "$T"
mv "/tmp/$REMOTE.csv" "$LOGS/${NAME}_${MODE}_${STAMP}_intro.csv"
echo "saved: $LOGS/${NAME}_${MODE}_${STAMP}_intro.csv"
