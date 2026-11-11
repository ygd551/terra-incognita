#!/bin/bash
# GBPlanner2 (ROS 1) exploring instead of TARE, all in the background. Normally started by ./explore.sh exploration:=gbplanner.
#   ros1_ws/gbplanner.sh [start]   roscore + GBPlanner2 + pci, ros1_bridge, isaac_adapter
#   ros1_ws/gbplanner.sh stop
# Env: ROBOT, LIDAR (default: the robot and lidar ./sim.sh was started with), RVIZ=false (no ROS 1 RViz with
#      GBPlanner's graph)
# Logs: docker exec gbplanner_ros1 tail -f /tmp/gbplanner_launch.log /tmp/isaac_adapter.log; docker logs gbplanner_bridge
set -e
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd)
R1="docker exec gbplanner_ros1"

if [ "${1:-}" = stop ]; then
  docker rm -f gbplanner_bridge >/dev/null 2>&1 || true
  # only GBPlanner2's processes: FAST-LIVO2 (fast_livo2.sh) runs in the same container on its own master (port 11312)
  $R1 pkill -INT -f "isaac_adapte[r]|gbplanner_isaac.launc[h]" 2>/dev/null || true; sleep 2
  $R1 pkill -9 -f "isaac_adapte[r]|gbplanner_isaac.launc[h]|rosmaste[r] --core -p 11311|gbplanner_nod[e]|pci_general_ros_nod[e]|config/gbplanner.rvi[z]" 2>/dev/null || true
  echo "gbplanner stopped"; exit 0
fi

# (devel/ holds symlinks into the container's /ws, dangling on the host: test the real binary)
[ -f "$HERE/devel/.private/gbplanner/lib/gbplanner/gbplanner_node" ] || { echo "build first: $HERE/ros1.sh build" >&2; exit 1; }
ROBOT=${ROBOT:-$(cat "$ROOT/logs/.robot" 2>/dev/null || echo anymal_d)}
LIDAR=${LIDAR:-$(cat "$ROOT/logs/.lidar" 2>/dev/null || echo os1_32)}
# GBPlanner2's lidar model is symmetric about the horizon: an asymmetric one (hdl32) keeps its total vertical span.
# Horizontal: 360 deg for spinning lidars, hfov for a Livox (looking forward)
read -r HFOV VFOV < <(python3 -c "import math, yaml; l = yaml.safe_load(open('$ROOT/config/lidars.yaml'))['$LIDAR']
print(math.radians(l.get('hfov', 360.0)), math.radians(l['vfov'][1] - l['vfov'][0]))")
# robot-specific planner and pci settings, per airframe (model in config/robots.yaml; the drone:
# config/gbplanner_config_rmf_obelix.yaml, pci_config_rmf_obelix.yaml)
MODEL=$(python3 -c "import yaml; print(yaml.safe_load(open('$ROOT/config/robots.yaml'))['$ROBOT'].get('model', '$ROBOT'))")
VARIANT=""; [ -f "$HERE/config/gbplanner_config_$MODEL.yaml" ] && VARIANT=_$MODEL
"$HERE/ros1.sh" start
"$0" stop >/dev/null
docker exec -d gbplanner_ros1 bash -c "source /ws/devel/setup.bash && \
  roslaunch /ws/launch/gbplanner_isaac.launch rviz:=${RVIZ:-true} hfov:=$HFOV vfov:=$VFOV variant:=$VARIANT \
  > /tmp/gbplanner_launch.log 2>&1"
for i in $(seq 30); do $R1 bash -c "source /opt/ros/noetic/setup.bash && rosparam get /use_sim_time" >/dev/null 2>&1 && break; sleep 1; done

# ros1_bridge (ROS 1 Noetic <-> ROS 2 Foxy, which talks to our Humble over DDS): only the topics in config/bridge.yaml
docker run -d --rm --name gbplanner_bridge --network host \
  -e ROS_MASTER_URI=http://localhost:11311 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml -v "$ROOT/config/fastdds.xml":/fastdds.xml:ro \
  -v "$HERE/config/bridge.yaml":/bridge.yaml:ro ros:foxy-ros1-bridge \
  bash -c "source /opt/ros/noetic/setup.bash && rosparam load /bridge.yaml && \
           source /opt/ros/foxy/setup.bash && exec ros2 run ros1_bridge parameter_bridge" >/dev/null
docker exec -d gbplanner_ros1 bash -c "source /ws/devel/setup.bash && \
  python3 -u /ws/scripts/isaac_adapter.py --robot $ROBOT > /tmp/isaac_adapter.log 2>&1"
echo "GBPlanner2 started for $ROBOT with lidar $LIDAR (logs: docker exec gbplanner_ros1 tail -f /tmp/gbplanner_launch.log /tmp/isaac_adapter.log)"
