#!/bin/bash
# Terminal 2 - the algorithms: BIEVR-LIO localization + mapping, elevation map, terrain analysis, TARE exploration,
# CMU local planner + path follower, RViz. Start it after ./sim.sh prints "[go2_sim] running".
#   ./explore.sh [lio:=bievr|gt] [speed:=1.0] [rviz:=false] [elevation:=false] [exploration:=false] [planner:=false]
#                [robot:=go2|anymal_d]
# robot:= defaults to the robot ./sim.sh was started with (they must match: lidar mount, footprint, heights).
set -e
ROOT=$(cd "$(dirname "$0")" && pwd)
[[ " $* " == *" robot:="* ]] || set -- robot:="$(cat "$ROOT/logs/.robot" 2>/dev/null || echo anymal_d)" "$@"
"$ROOT/container.sh" start
docker exec isaac_explore pgrep -f "go2_si[m].py" >/dev/null || echo "warning: no simulation running - start ./sim.sh first" >&2
exec docker exec -it isaac_explore bash -c "source /isaac_explore/docker/algo_env.sh && \
  ros2 launch /isaac_explore/launch/explore.launch.py $(printf '%q ' "$@")"
