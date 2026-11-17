#!/usr/bin/env bash
# Compare sim vs real for the whole standard test set (the standard lab set), one block per test.
# Normally started with ./fr3 cmp-all  (copy of crisp_lab/bin/cmp_all.sh; logs are in data/logs).
#   ./fr3 cmp-all                    # gains tag k300_15
#   GAINS=k600_20 ./fr3 cmp-all      # another gain step (the tag you put in the --log names)
#   CMP_B=sim ./fr3 cmp-all          # smoke test: sim against sim
# Tests with no real log yet are skipped.  Output is also saved to data/logs/cmp/summary_<time>.txt
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="${FR3_LOG_DIR:-/home/ros/data/logs}"
G="${GAINS:-k300_15}"
mkdir -p "$LOGS/cmp"
OUT="$LOGS/cmp/summary_$(date +%Y%m%d_%H%M%S).txt"
run() { echo; echo "######## $*"; "$HERE/cmp.sh" "$@" 2>&1 || echo "(skipped)"; }
{
  run hold_$G --tag hold
  run holdside_p_$G --tag hold
  run holdside_n_$G --tag hold
  run wp_$G --tag settle
  for v in 0.02 0.04 0.06; do run circle_v${v}_$G --tag trace; done
  run fig8_$G --tag trace
  for ax in x y z rx ry rz; do run db_${ax}_$G --steps; done
  for j in 1 2 3 4 5 6 7; do run dbj${j}_$G --steps; run dbj${j}_fine_$G --steps; done
} 2>&1 | tee "$OUT"
echo; echo "saved: $OUT   (plots in $LOGS/cmp/)"
