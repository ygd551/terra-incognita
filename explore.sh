#!/bin/bash
# Terminal 2 - the algorithms: BIEVR-LIO localization + mapping, elevation map, terrain analysis, exploration,
# CMU local planner + path follower, RViz. Start it after ./sim.sh prints "[go2_sim] running".
#   ./explore.sh [lio:=bievr|gt|fastlivo2] [speed:=1.0] [rviz:=false] [elevation:=false] [exploration:=tare|gbplanner|false]
#                [planner:=false] [robot:=go2|anymal_d|rmf_obelix_ouster64|rmf_obelix_avia]
# robot:= defaults to the robot ./sim.sh was started with (they must match: lidar mount, footprint, heights).
# exploration:=gbplanner explores with GBPlanner2 (ROS 1 container + ros1_bridge, ros1_ws/) instead of TARE.
# lio:=fastlivo2 localizes and maps with FAST-LIVO2 instead of BIEVR-LIO (ROS 1 container, ros1_ws/fast_livo2.sh): a
# colored map in its own RViz window with camera images (the drone, sim without --no_camera), saved when this stops;
# that RViz also shows the way point, so the ROS 2 RViz stays closed unless rviz:=true is given.
# A drone (./sim.sh --robot rmf_obelix_ouster64|rmf_obelix_avia) explores with GBPlanner2 by default; exploration:=false
# = fly it yourself.
set -e
ROOT=$(cd "$(dirname "$0")" && pwd)
[[ " $* " == *" robot:="* ]] || set -- robot:="$(cat "$ROOT/logs/.robot" 2>/dev/null || echo anymal_d)" "$@"
EXPLORER=tare; ARGS=(); FLIVO=false
for a in "$@"; do
  case $a in
    exploration:=gbplanner) EXPLORER=gbplanner; ARGS+=(exploration:=false) ;;
    exploration:=tare) ARGS+=(exploration:=true) ;;
    robot:=*) ROBOT=${a#robot:=}; ARGS+=("$a") ;;
    rviz:=false) RVIZ=false; ARGS+=("$a") ;;
    lio:=fastlivo2) FLIVO=true; ARGS+=("$a") ;;
    *) ARGS+=("$a") ;;
  esac
done
$FLIVO && [[ " $* " != *" rviz:="* ]] && ARGS+=(rviz:=false)   # FAST-LIVO2's RViz has the map and the way point
if sed -n "/^$ROBOT:/,/^[^ ]/p" "$ROOT/config/robots.yaml" | grep -q "^  type: aerial"; then
  [[ " $* " == *" exploration:=tare "* ]] && { echo "TARE plans for ground robots: a drone explores with GBPlanner2" >&2; exit 1; }
  [[ " $* " == *" exploration:="* ]] || { EXPLORER=gbplanner; ARGS+=(exploration:=false); }
fi
"$ROOT/container.sh" start
docker exec isaac_explore pgrep -f "go2_si[m].py|drone_si[m].py" >/dev/null || echo "warning: no simulation running - start ./sim.sh first" >&2
CMD="source /isaac_explore/docker/algo_env.sh && ros2 launch /isaac_explore/launch/explore.launch.py $(printf '%q ' "${ARGS[@]}")"
[ $EXPLORER = tare ] && ! $FLIVO && exec docker exec -it isaac_explore bash -c "$CMD"

GBP=$ROOT/ros1_ws/gbplanner.sh; FL2=$ROOT/ros1_ws/fast_livo2.sh
cleanup() { [ $EXPLORER = gbplanner ] && "$GBP" stop; $FLIVO && "$FL2" stop; true; }   # FAST-LIVO2 writes its map
trap cleanup EXIT
if $FLIVO; then ROBOT=$ROBOT RVIZ=${RVIZ:-true} "$FL2" start; fi
if [ $EXPLORER = gbplanner ]; then
  [ -x "$GBP" ] || { echo "exploration:=gbplanner needs $GBP" >&2; exit 1; }
  ROBOT=$ROBOT RVIZ=${RVIZ:-true} "$GBP" start
fi
docker exec -it isaac_explore bash -c "$CMD"
