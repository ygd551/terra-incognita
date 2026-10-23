#!/bin/bash
# The project's single container (image isaac_explore): Isaac Sim + Isaac Lab + ROS 2 + all algorithms.
# sim.sh / explore.sh / run_trial.sh call it; by hand:
#   ./container.sh          start it if needed and open a shell (for the algorithms: source docker/algo_env.sh)
#   ./container.sh start    only start it
#   ./container.sh build    colcon-build ros2_ws inside it (incremental)
#   ./container.sh rm       remove it (Isaac's shader / asset caches survive in docker volumes)
set -e
ROOT=$(cd "$(dirname "$0")" && pwd)
NAME=isaac_explore
# Memory cap of the container: if Isaac Sim or a build overruns it, only processes in the container are killed instead
# of the whole desktop freezing. Default: total RAM - 9 GB for the desktop, clamped to 16..26 GB (22 GB on a 32 GB
# machine). The first Isaac Sim start compiles the RTX render pipelines and peaks at ~21 GB; later starts use ~5 GB.
MEM=${ISAAC_EXPLORE_MEM:-$(awk '/MemTotal/ {m = int($2 / 1048576) - 9; print (m < 16 ? 16 : (m > 26 ? 26 : m)) "g"}' /proc/meminfo)}

start() {
  xhost +si:localuser:root >/dev/null 2>&1 || true   # GUI windows (Isaac Sim, RViz); X forgets it on every restart
  local src
  src=$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/isaac_explore"}}{{.Source}}{{end}}{{end}}' $NAME 2>/dev/null || true)   # none yet: empty
  if [ -n "$src" ] && [ "$src" != "$ROOT" ]; then   # another checkout of the project owns the container
    echo "container $NAME was started from $src: recreating it for $ROOT" >&2
    docker rm -f $NAME >/dev/null
  fi
  docker ps --format '{{.Names}}' | grep -qx $NAME && return
  docker rm -f $NAME >/dev/null 2>&1 || true
  local vols=() v
  for v in cache-kit:/isaac-sim/kit/cache cache-ov:/root/.cache/ov cache-gl:/root/.cache/nvidia/GLCache \
           cache-compute:/root/.nv/ComputeCache data:/root/.local/share/ov/data; do
    vols+=(-v "isaac_explore_$v")
  done
  # --init reaps the zombies that ros2 launch leaves behind
  docker run -d --init --name $NAME --gpus all --network host --memory "$MEM" --memory-swap "$MEM" \
    -e ACCEPT_EULA=Y -e OMNI_KIT_ALLOW_ROOT=1 -e ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim \
    -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$ROOT":/isaac_explore "${vols[@]}" --entrypoint sleep isaac_explore infinity >/dev/null
}

case "${1:-shell}" in
  start) start ;;
  build)
    start
    # 2 packages x JOBS compilers at a time, ~1.5 GB each (PCL-heavy C++); more is not much faster and can hit the cap
    docker exec $NAME bash -c "source /opt/ros/humble/setup.bash && cd /isaac_explore/ros2_ws && \
      MAKEFLAGS=-j${ISAAC_EXPLORE_JOBS:-4} colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release"
    docker exec $NAME chown -R "$(id -u):$(id -g)" /isaac_explore/ros2_ws/build /isaac_explore/ros2_ws/install \
      /isaac_explore/ros2_ws/log ;;
  rm) docker rm -f $NAME ;;
  shell) start; exec docker exec -it $NAME bash ;;
  *) sed -n 2,7p "$0"; exit 1 ;;
esac
