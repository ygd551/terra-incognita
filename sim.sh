#!/bin/bash
# Terminal 1 - the simulation: Isaac Sim with the robot, its walking policy, lidar + IMU and the ROS 2 interface.
#   ./sim.sh [--scene NAME] [--robot anymal_d|go2|rmf_obelix_ouster64|rmf_obelix_avia] [--lidar NAME] [--spawn X Y [Z]]
#            [--yaw DEG] [--headless] ...
#                                   (lidar models: config/lidars.yaml; default: the robot's `lidar` in
#                                    config/robots.yaml, else os1_32)
#   --robot rmf_obelix_ouster64 / rmf_obelix_avia: the RMF-Obelix multirotor with an Ouster OS0-64 / a Livox Avia
#                                   (sim/drone_sim.py; asset: tools/add_drone.sh)
#   ./sim.sh --list                 the scenes in sim/scenes/ (start poses: config/scenes.yaml)
# (all options: sim/go2_sim.py --help). Ready when it prints "[go2_sim] running"; then run ./explore.sh in terminal 2.
set -e
ROOT=$(cd "$(dirname "$0")" && pwd)
if [ "${1:-}" = --list ]; then
  for f in "$ROOT"/sim/scenes/*.usd; do n=$(basename "$f" .usd)
    printf "%-28s %s\n" "$n" "$(grep -E "^$n:" "$ROOT/config/scenes.yaml" | cut -d' ' -f2- || true)"; done
  exit 0
fi
ROBOT=anymal_d; SCENE=simple_cave_01; LIDAR=
for ((i = 1; i <= $#; i++)); do j=$((i + 1))
  [ "${!i}" = --robot ] && ROBOT=${!j}
  [ "${!i}" = --scene ] && SCENE=${!j}
  [ "${!i}" = --lidar ] && LIDAR=${!j}
done
[[ " $* " == *" --robot "* ]] || set -- --robot $ROBOT "$@"
# aerial robots (type: aerial in config/robots.yaml) fly in drone_sim.py; the lidar defaults to the robot's `lidar`
SIM=go2_sim.py; RCFG=$(sed -n "/^$ROBOT:/,/^[^ ]/p" "$ROOT/config/robots.yaml")
[ -n "$RCFG" ] || { echo "no robot $ROBOT, one of: $(grep -oE '^[a-z0-9_]+:' "$ROOT/config/robots.yaml" | tr -d : | xargs)" >&2; exit 1; }
if echo "$RCFG" | grep -q "^  type: aerial"; then SIM=drone_sim.py; fi
[ -n "$LIDAR" ] || { LIDAR=$(echo "$RCFG" | sed -n 's/^  lidar: *\([a-z0-9_]*\).*/\1/p'); LIDAR=${LIDAR:-os1_32}
                     set -- --lidar $LIDAR "$@"; }
SCENE_NAME=$(basename "$SCENE" .usd)
[[ "$SCENE" == *.usd ]] || [ -f "$ROOT/sim/scenes/$SCENE.usd" ] || {
  echo "no scene sim/scenes/$SCENE.usd (./sim.sh --list; simple_cave_01 comes with ./setup.sh)" >&2; exit 1; }
grep -q "^$LIDAR:" "$ROOT/config/lidars.yaml" || {
  echo "no lidar $LIDAR, one of: $(grep -oE '^[a-z0-9_]+:' "$ROOT/config/lidars.yaml" | tr -d : | xargs)" >&2; exit 1; }
"$ROOT/container.sh" start
# two sims on one ROS 2 network silently corrupt each other's topics
if docker exec isaac_explore pgrep -f "go2_si[m].py|drone_si[m].py" >/dev/null; then
  echo "a simulation is already running (./stop_all.sh stops it)" >&2; exit 1
fi
mkdir -p "$ROOT/logs" && echo "$ROBOT" > "$ROOT/logs/.robot"   # explore.sh starts the stack for the same robot
echo "$SCENE_NAME" > "$ROOT/logs/.scene"                        # for the exploration metrics (tile coverage)
echo "$LIDAR" > "$ROOT/logs/.lidar"                             # GBPlanner2's sensor model (ros1_ws/gbplanner.sh)
exec docker exec -it isaac_explore bash -c "source /opt/ros/humble/setup.bash && cd /workspace/isaaclab && \
  ./isaaclab.sh -p /workspace/isaac_explore/sim/$SIM $(printf '%q ' "$@")"
