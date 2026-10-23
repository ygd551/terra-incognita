#!/bin/bash
# One unattended exploration trial: headless sim + full stack, recorded until TARE reports completion or a
# sim-time limit, then a TARE state snapshot. Results in logs/trial_<name>/.
#   [LAUNCH_ARGS="lio:=gt ..."] ./run_trial.sh NAME [MAX_SIM_SECONDS=1200] [extra go2_sim.py args, e.g. --robot go2]
set -u
# one trial at a time: a finishing trial runs stop_all.sh, which would kill the next trial's sim mid-startup
# (pid file, not flock: an flock fd is inherited by child sleeps / docker execs and outlives a killed script)
LOCK=/tmp/isaac_explore_run_trial.pid
if [ -f $LOCK ] && kill -0 "$(cat $LOCK)" 2>/dev/null; then
  echo "[trial] another run_trial.sh (pid $(cat $LOCK)) is still running; wait for it or stop it first" >&2; exit 1
fi
echo $$ > $LOCK; trap 'rm -f $LOCK' EXIT
NAME=$1; MAX=${2:-1200}; shift; shift || true
ROBOT=anymal_d; LIDAR=os1_32   # same defaults as sim.sh; the stack must be started for the simulated robot
for ((i = 1; i <= $#; i++)); do j=$((i + 1))
  [ "${!i}" = --robot ] && ROBOT=${!j}
  [ "${!i}" = --lidar ] && LIDAR=${!j}
done
[[ " $* " == *" --robot "* ]] || set -- --robot $ROBOT "$@"
LAUNCH_ARGS="robot:=$ROBOT ${LAUNCH_ARGS:-}"
cd "$(dirname "$0")" && ./container.sh start && ./stop_all.sh >/dev/null
OUT=logs/trial_$NAME; mkdir -p "$OUT"; C="docker exec isaac_explore"; IN=/isaac_explore/$OUT
echo "$ROBOT" > logs/.robot; echo "$LIDAR" > logs/.lidar   # as sim.sh: read by ros1_ws/gbplanner.sh
docker exec -d isaac_explore bash -c "source /opt/ros/humble/setup.bash; cd /workspace/isaaclab; PYTHONUNBUFFERED=1 \
  ./isaaclab.sh -p /workspace/isaac_explore/sim/go2_sim.py --headless $* > /tmp/go2_sim.log 2>&1"
for i in $(seq 60); do sleep 5; $C grep -aqE "\[go2_sim\] running|Traceback" /tmp/go2_sim.log && break; done
sleep 10
docker exec -d isaac_explore bash -c "source /isaac_explore/docker/algo_env.sh; \
  ros2 launch /isaac_explore/launch/explore.launch.py rviz:=false elevation:=false $LAUNCH_ARGS > /tmp/explore.log 2>&1"
sleep 15
docker exec -d isaac_explore bash -c "source /isaac_explore/docker/algo_env.sh; \
  python3 /isaac_explore/sim/record_run.py $IN/run.csv $MAX > $IN/record.log 2>&1"
# wait for completion or the recorder to finish
while $C pgrep -f "record_run.py $IN/run.cs[v]" >/dev/null; do
  $C grep -aq "Exploration completed" /tmp/explore.log && { echo "[trial] TARE reports exploration completed"; break; }
  sleep 20
done
$C bash -c "source /isaac_explore/docker/algo_env.sh; timeout 60 python3 /isaac_explore/sim/tare_state.py" > "$OUT/tare_state.txt" 2>&1
$C pkill -INT -f "record_run.py $IN/run.cs[v]"; sleep 3
$C bash -c "cp /tmp/explore.log $IN/explore.log; f=\$(ls -t /isaac_explore/logs/metrics_*.txt | head -1); cp \$f $IN/metrics.txt"
$C cat /tmp/go2_sim.log | grep -a "\[go2_sim\]" > "$OUT/go2_sim_events.txt"
./stop_all.sh >/dev/null
echo "[trial] $NAME done -> $OUT"; grep tare "$OUT/tare_state.txt"; tail -1 "$OUT/metrics.txt"
