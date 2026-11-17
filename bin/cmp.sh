#!/usr/bin/env bash
# Compare the LATEST sim log and the LATEST real log of the same test name (offline, no ROS needed).
# Normally started with ./fr3 cmp NAME [--tag TAG] [--steps]  (copy of crisp_lab/bin/cmp.sh; logs are in data/logs).
#   ./fr3 cmp hold_k300_15 --tag hold
#   ./fr3 cmp circle_v0.04_k300_15 --tag trace
#   ./fr3 cmp db_rz_k300_15 --steps          # dead-band tests (the *_steps.csv files)
# NAME is the --log name you passed to the script (the gains are part of it). A PNG goes to data/logs/cmp/NAME.png.
# For a smoke test on sim logs only:  CMP_B=sim ./fr3 cmp hold_k300_15 --tag hold
set -euo pipefail
LOGS="${FR3_LOG_DIR:-/home/ros/data/logs}"
NAME="${1:?usage: cmp.sh NAME [--tag TAG] [--steps]}"; shift
A="${CMP_A:-sim}"; B="${CMP_B:-real}"
STEPS=0; ARGS=()
for x in "$@"; do if [ "$x" = "--steps" ]; then STEPS=1; else ARGS+=("$x"); fi; done
if [ "$STEPS" = 1 ]; then SUF="_????????_??????_steps.csv"; else SUF="_????????_??????.csv"; fi
FA="$(ls -t "$LOGS"/${NAME}_${A}${SUF} 2>/dev/null | head -1 || true)"
FB="$(ls -t "$LOGS"/${NAME}_${B}${SUF} 2>/dev/null | head -1 || true)"
[ -n "$FA" ] || { echo "no $A log for '$NAME' in $LOGS"; exit 2; }
[ -n "$FB" ] || { echo "no $B log for '$NAME' in $LOGS"; exit 2; }
for d in /ws_override/scripts /opt/asu_fr3/scripts; do [ -f "$d/06_compare_logs.py" ] && { S="$d/06_compare_logs.py"; break; }; done
mkdir -p "$LOGS/cmp"
echo "A: $(basename "$FA")"; echo "B: $(basename "$FB")"
python3 "$S" "$FA" "$FB" ${ARGS[@]+"${ARGS[@]}"} --png "$LOGS/cmp/${NAME}.png"
