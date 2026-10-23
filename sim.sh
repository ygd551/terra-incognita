#!/bin/bash
# Terminal 1 - the simulation: Isaac Sim with the robot, its walking policy, lidar + IMU and the ROS 2 interface.
#   ./sim.sh [--scene NAME] [--robot anymal_d|go2] [--spawn X Y [Z]] [--yaw DEG] [--headless] [--see_through 0.25] ...
#   ./sim.sh --list                 the scenes in sim/scenes/ (start poses: config/scenes.yaml)
# (all options: sim/go2_sim.py --help). Ready when it prints "[go2_sim] running"; then run ./explore.sh in terminal 2.
set -e
ROOT=$(cd "$(dirname "$0")" && pwd)
if [ "${1:-}" = --list ]; then
  for f in "$ROOT"/sim/scenes/*.usd; do n=$(basename "$f" .usd)
    printf "%-28s %s\n" "$n" "$(grep -E "^$n:" "$ROOT/config/scenes.yaml" | cut -d' ' -f2- || true)"; done
  exit 0
fi
ROBOT=anymal_d; SCENE=simple_cave_01
for ((i = 1; i <= $#; i++)); do j=$((i + 1))
  [ "${!i}" = --robot ] && ROBOT=${!j}
  [ "${!i}" = --scene ] && SCENE=${!j}
done
[[ " $* " == *" --robot "* ]] || set -- --robot $ROBOT "$@"
SCENE_NAME=$(basename "$SCENE" .usd)
[[ "$SCENE" == *.usd ]] || [ -f "$ROOT/sim/scenes/$SCENE.usd" ] || {
  echo "no scene sim/scenes/$SCENE.usd (./sim.sh --list; simple_cave_01 comes with ./setup.sh)" >&2; exit 1; }
"$ROOT/container.sh" start
# two sims on one ROS 2 network silently corrupt each other's topics
if docker exec isaac_explore pgrep -f "go2_si[m].py" >/dev/null; then
  echo "a simulation is already running (./stop_all.sh stops it)" >&2; exit 1
fi
mkdir -p "$ROOT/logs" && echo "$ROBOT" > "$ROOT/logs/.robot"   # explore.sh starts the stack for the same robot
echo "$SCENE_NAME" > "$ROOT/logs/.scene"                        # for the exploration metrics (tile coverage)
exec docker exec -it isaac_explore bash -c "source /opt/ros/humble/setup.bash && cd /workspace/isaaclab && \
  ./isaaclab.sh -p /workspace/isaac_explore/sim/go2_sim.py $(printf '%q ' "$@")"
