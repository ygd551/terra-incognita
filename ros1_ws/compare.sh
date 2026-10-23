#!/bin/bash
# TARE vs GBPlanner2: N unattended trials each, alternating, under identical conditions (headless sim, ground-truth
# localization, same sim-time budget, --auto_reset, the same lower layers). Then the coverage of every trial.
#   ros1_ws/compare.sh [N=3] [MAX_SIM_SECONDS=1200]     results: logs/trial_{tare,gbp}_<tag>_<i>/, logs/compare_<tag>.txt
set -u
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd)
N=${1:-3}; MAX=${2:-1200}; TAG=$(date +%m%d_%H%M)
for i in $(seq "$N"); do
  LAUNCH_ARGS="lio:=gt" "$ROOT/run_trial.sh" "tare_${TAG}_$i" "$MAX" --auto_reset 10
  LAUNCH_ARGS="lio:=gt" "$HERE/run_trial.sh" "${TAG}_$i" "$MAX" --auto_reset 10
done
for d in "$ROOT"/logs/trial_tare_"$TAG"_* "$ROOT"/logs/trial_gbp_"$TAG"_*; do
  echo "== $(basename "$d"): $(grep -c 'has fallen' "$d/go2_sim_events.txt" 2>/dev/null) falls, $(grep -c 'stuck\|found no path' "$d/go2_sim_events.txt" 2>/dev/null) stuck/no-path recoveries"
  python3 "$HERE/scripts/tile_coverage.py" "$d/run.csv" "$d/metrics.txt" --step 300 --until "$MAX" 2>&1 | tail -n +2
done | tee "$ROOT/logs/compare_$TAG.txt"
