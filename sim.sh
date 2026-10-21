#!/bin/bash
# Terminal 1 - the simulation: Isaac Sim with the robot, its walking policy, lidar + IMU and the ROS 2 interface.
#   ./sim.sh [--robot anymal_d|go2] [--headless] [--scene sim/scenes/NAME.usd] [--spawn X Y Z] [--see_through 0.25] ...
# (all options: sim/go2_sim.py --help). Ready when it prints "[go2_sim] running"; then run ./explore.sh in terminal 2.
set -e
ROOT=$(cd "$(dirname "$0")" && pwd)
ROBOT=anymal_d
for ((i = 1; i <= $#; i++)); do [ "${!i}" = --robot ] && { j=$((i + 1)); ROBOT=${!j}; }; done
[[ " $* " == *" --robot "* ]] || set -- --robot $ROBOT "$@"
[ -f "$ROOT/sim/scenes/simple_cave_01.usd" ] || { echo "sim/scenes/simple_cave_01.usd is missing: run ./setup.sh first" >&2; exit 1; }
"$ROOT/container.sh" start
# two sims on one ROS 2 network silently corrupt each other's topics
if docker exec isaac_explore pgrep -f "go2_si[m].py" >/dev/null; then
  echo "a simulation is already running (./stop_all.sh stops it)" >&2; exit 1
fi
mkdir -p "$ROOT/logs" && echo "$ROBOT" > "$ROOT/logs/.robot"   # explore.sh starts the stack for the same robot
exec docker exec -it isaac_explore bash -c "source /opt/ros/humble/setup.bash && cd /workspace/isaaclab && \
  ./isaaclab.sh -p /workspace/isaac_explore/sim/go2_sim.py $(printf '%q ' "$@")"
