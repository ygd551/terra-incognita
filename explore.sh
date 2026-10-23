#!/bin/bash
# Terminal 2 - the algorithms: BIEVR-LIO localization + mapping, elevation map, terrain analysis, exploration,
# CMU local planner + path follower, RViz. Start it after ./sim.sh prints "[go2_sim] running".
#   ./explore.sh [lio:=bievr|gt] [speed:=1.0] [rviz:=false] [elevation:=false] [exploration:=tare|gbplanner|false]
#                [planner:=false] [robot:=go2|anymal_d]
# robot:= defaults to the robot ./sim.sh was started with (they must match: lidar mount, footprint, heights).
# exploration:=gbplanner explores with GBPlanner2 (ROS 1 container + ros1_bridge, ros1_ws/) instead of TARE.
set -e
ROOT=$(cd "$(dirname "$0")" && pwd)
[[ " $* " == *" robot:="* ]] || set -- robot:="$(cat "$ROOT/logs/.robot" 2>/dev/null || echo anymal_d)" "$@"
EXPLORER=tare; ARGS=()
for a in "$@"; do
  case $a in
    exploration:=gbplanner) EXPLORER=gbplanner; ARGS+=(exploration:=false) ;;
    exploration:=tare) ARGS+=(exploration:=true) ;;
    robot:=*) ROBOT=${a#robot:=}; ARGS+=("$a") ;;
    rviz:=false) RVIZ=false; ARGS+=("$a") ;;
    *) ARGS+=("$a") ;;
  esac
done
"$ROOT/container.sh" start
docker exec isaac_explore pgrep -f "go2_si[m].py" >/dev/null || echo "warning: no simulation running - start ./sim.sh first" >&2
CMD="source /isaac_explore/docker/algo_env.sh && ros2 launch /isaac_explore/launch/explore.launch.py $(printf '%q ' "${ARGS[@]}")"
[ $EXPLORER = tare ] && exec docker exec -it isaac_explore bash -c "$CMD"

GBP=$ROOT/ros1_ws/gbplanner.sh
[ -x "$GBP" ] || { echo "exploration:=gbplanner needs $GBP" >&2; exit 1; }
ROBOT=$ROBOT RVIZ=${RVIZ:-true} "$GBP" start
trap '"$GBP" stop' EXIT
docker exec -it isaac_explore bash -c "$CMD"
