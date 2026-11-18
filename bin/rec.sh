#!/usr/bin/env bash
# Record the controller's 1 kHz introspection stream (torques, errors, joint state, missed cycles) to a CSV.
# Runs INSIDE the stack container; normally started with ./fr3 rec [mode] NAME SECONDS  (copy of crisp_lab/bin/rec.sh).
# Columns: t, stamp, qN, dqN, tau_measN (reported torque), tau_cmdN (commanded torque), cmd_limitedN, cycles_/missed_<controller> (see tools/rec_introspection.py).
# enable_introspection: true in the cartesian block of the controllers file adds the cartesian_impedance_controller.* columns (not needed for the rest).
#   ./fr3 rec mujoco hold_side 30        # -> data/logs/hold_side_sim_<time>_intro.csv
#   ./fr3 rec real   hold_side 30
# Start it a second or two BEFORE the test, give it a few seconds more than the test lasts, or press Ctrl+C when the test is done (the file is still saved).
# The file is written while recording, so a long session needs no memory and a killed recording keeps what was written.
set -euo pipefail
trap ':' INT      # Ctrl+C reaches python (it stops and finishes the file); this script then carries on
NAME="${1:?usage: rec.sh NAME SECONDS}"; SECS="${2:?SECONDS}"
MODE="${CRISP_LAB_MODE:-sim}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOGS="${FR3_LOG_DIR:-/home/ros/data/logs}"; mkdir -p "$LOGS"
FINAL="$LOGS/${NAME}_${MODE}_${STAMP}_intro.csv"
for d in /ws_override/tools /opt/asu_fr3/tools; do [ -f "$d/rec_introspection.py" ] && { T="$d/rec_introspection.py"; break; }; done
python3 - "$FINAL" "$SECS" < "$T"      # writes the file while recording (a kill keeps what was written)
echo "saved: $FINAL"
