#!/bin/bash
# One unattended GBPlanner2 trial: the project's run_trial.sh without TARE (exploration:=false), plus gbplanner.sh
# once the stack is up. Results in logs/trial_gbp_<NAME>/ (+ gbplanner.log, gbplanner_launch.log, isaac_adapter.log).
#   [LAUNCH_ARGS="lio:=gt"] ros1_ws/run_trial.sh NAME [MAX_SIM_SECONDS=1200] [extra go2_sim.py args]
#   then: python3 ros1_ws/scripts/tile_coverage.py logs/trial_gbp_NAME/run.csv logs/trial_gbp_NAME/metrics.txt
set -u
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd)
NAME=gbp_$1; shift; OUT=$ROOT/logs/trial_$NAME
LAUNCH_ARGS="exploration:=false ${LAUNCH_ARGS:-}" "$ROOT/run_trial.sh" "$NAME" "$@" &
TRIAL=$!
# the recorder starts 15 s after the stack's launch
until docker exec isaac_explore pgrep -f "record_run.py /isaac_explore/logs/trial_$NAME/run.cs[v]" >/dev/null 2>&1; do
  kill -0 $TRIAL 2>/dev/null || exit 1; sleep 5
done
RVIZ=false "$HERE/gbplanner.sh" start > "$OUT/gbplanner.log" 2>&1
wait $TRIAL
docker cp gbplanner_ros1:/tmp/gbplanner_launch.log "$OUT/" 2>/dev/null
docker cp gbplanner_ros1:/tmp/isaac_adapter.log "$OUT/" 2>/dev/null
"$HERE/gbplanner.sh" stop >/dev/null
