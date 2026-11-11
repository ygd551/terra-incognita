#!/bin/bash
# The GBPlanner2 baseline's ROS 1 Noetic container (image gbplanner_ros1 from docker/Dockerfile).
#   ./ros1.sh           start it if needed and open a shell (ROS 1 + the built workspace sourced)
#   ./ros1.sh start     only start it
#   ./ros1.sh build     catkin-build ws/ inside it
#   ./ros1.sh rm        remove it
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
NAME=gbplanner_ros1
MEM=${GBPLANNER_MEM:-10g}   # cap: a runaway build or planner only kills processes in here
# shared third-party tree (FAST-LIVO2's Sophus and rpg_vikit, see fetch_src.sh): mounted read-only at the same path, so
# absolute paths (library locations, the src/rpg_vikit link) mean the same inside and outside
TP=${THIRDPARTY_ROOT:-$HOME/thirdparty}

start() {
  xhost +si:localuser:root >/dev/null 2>&1 || true   # RViz window
  local src
  src=$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/ws"}}{{.Source}}{{end}}{{end}}' $NAME 2>/dev/null || true)   # none yet: empty
  if [ -n "$src" ] && [ "$src" != "$HERE" ]; then   # another checkout of the project owns the container
    echo "container $NAME was started from $src: recreating it for $HERE" >&2
    docker rm -f $NAME >/dev/null
  elif [ -n "$src" ] && [ -d "$TP" ] && ! docker inspect -f '{{range .Mounts}}{{.Destination}} {{end}}' $NAME | grep -qw "$TP"; then
    echo "container $NAME has no $TP mount: recreating it" >&2
    docker rm -f $NAME >/dev/null
  fi
  docker ps --format '{{.Names}}' | grep -qx $NAME && return
  docker rm -f $NAME >/dev/null 2>&1 || true
  docker run -d --init --name $NAME --gpus all --network host --memory "$MEM" --memory-swap "$MEM" \
    -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HERE":/ws -v "$(cd "$HERE/.." && pwd)/config":/isaac_explore_config:ro $([ -d "$TP" ] && echo "-v $TP:$TP:ro") \
    --entrypoint sleep gbplanner_ros1 infinity >/dev/null
}

case "${1:-shell}" in
  start) start ;;
  build)
    start
    # 2 packages x JOBS compilers at a time (~1.5 GB each)
    docker exec $NAME bash -c "source /opt/ros/noetic/setup.bash && cd /ws && \
      catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release -DSophus_DIR=/ws/cmake \
        -DTHIRDPARTY_ROOT=$TP >/dev/null && \
      catkin build -j${ISAAC_EXPLORE_JOBS:-4} -p 2 --no-status --summarize"
    docker exec $NAME chown -R "$(id -u):$(id -g)" /ws ;;
  rm) docker rm -f $NAME ;;
  shell) start; exec docker exec -it $NAME bash ;;
  *) sed -n 2,7p "$0"; exit 1 ;;
esac
