#!/bin/bash
# One-time installation of everything (see README). Safe to re-run: finished steps are skipped or cached, so after a
# network failure just run it again.
#   ./setup.sh [--cn]        --cn: use Chinese mirrors (Tsinghua) for apt inside the image build
# Optional: ISAACLAB_DIR=/path/to/IsaacLab (default third_party/IsaacLab) to reuse an existing v2.1.1 checkout.
#           ISAAC_EXPLORE_JOBS=N parallel C++ compile jobs (default 4, ~1.5 GB RAM each)
set -eo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd); cd "$ROOT"
ISAACLAB_DIR=${ISAACLAB_DIR:-$ROOT/third_party/IsaacLab}
ISAACSIM_IMAGE=nvcr.io/nvidia/isaac-sim:4.5.0
MIRROR_ARGS=()
[ "${1:-}" = --cn ] && MIRROR_ARGS=(--build-arg UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
                                    --build-arg ROS_APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu)
step() { echo; echo "=== [$1/7] $2"; }
fail() { echo "ERROR: $*" >&2; exit 1; }
have_image() { docker image inspect "$1" >/dev/null 2>&1; }

step 1 "Check the host"
command -v nvidia-smi >/dev/null || fail "no NVIDIA driver found (>= 535 needed)"
DRIVER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
[ "${DRIVER%%.*}" -ge 535 ] || fail "NVIDIA driver $DRIVER is too old: Isaac Sim 4.5 needs >= 535.129"
for c in docker git curl python3; do command -v $c >/dev/null || fail "$c is missing"; done
docker info >/dev/null 2>&1 || fail "this user cannot run docker: sudo usermod -aG docker \$USER, then log in again"
docker info 2>/dev/null | grep -qi nvidia || fail "NVIDIA Container Toolkit is not installed:
  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html"
FREE=$(df -BG --output=avail "$(docker info -f '{{.DockerRootDir}}' 2>/dev/null || echo /)" 2>/dev/null | tail -1 | tr -dc 0-9)
have_image isaac-lab-base || [ "${FREE:-999}" -ge 70 ] || fail "only ${FREE} GB free on the Docker disk; the first installation needs about 70 GB"
echo "driver $DRIVER, ${FREE:-?} GB free for Docker"

step 2 "Third-party ROS 2 sources (versions pinned in ros2_ws/src.repos + ros2_ws/patches)"
ros2_ws/fetch_src.sh

step 3 "Isaac Sim 4.5 + Isaac Lab v2.1.1 base image (first time: ~25 GB download)"
if have_image isaac-lab-base; then
  echo "already there, skipped"
else
  if [ ! -d "$ISAACLAB_DIR/.git" ]; then
    GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 -b v2.1.1 https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB_DIR"
  fi
  if git -C "$ISAACLAB_DIR" apply --check -R "$ROOT/docker/isaaclab.patch" 2>/dev/null; then
    echo "docker/isaaclab.patch already applied"
  else
    git -C "$ISAACLAB_DIR" apply "$ROOT/docker/isaaclab.patch" || fail "docker/isaaclab.patch does not apply: is $ISAACLAB_DIR Isaac Lab v2.1.1?"
  fi
  # the Isaac Sim image alone is 15 GB: fall back to a resumable download if docker pull keeps failing
  have_image $ISAACSIM_IMAGE || docker pull $ISAACSIM_IMAGE || tools/fetch_image.sh nvidia/isaac-sim 4.5.0
  (cd "$ISAACLAB_DIR/docker" && docker compose --env-file .env.base --file docker-compose.yaml --profile base build isaac-lab-base)
fi

step 4 "Project image isaac_explore (ROS 2 + algorithm dependencies; seconds when nothing changed)"
docker/fetch_wheels.sh | grep -v "^ok "
docker build --network host -t isaac_explore -f docker/Dockerfile --build-arg BUILD_JOBS="${ISAAC_EXPLORE_JOBS:-4}" \
  "${MIRROR_ARGS[@]}" .

step 5 "Download assets: walking-policy checkpoints, SubT cave"
python3 tools/fetch_assets.py

step 6 "Convert the scene SDF -> USD"
if [ "$(docker inspect -f '{{.Image}}' isaac_explore 2>/dev/null)" != "$(docker image inspect -f '{{.Id}}' isaac_explore)" ]; then
  ./container.sh rm >/dev/null 2>&1 || true   # container from an older image: recreate it
fi
./container.sh start
if [ -f sim/scenes/simple_cave_01.usd ]; then
  echo "already there, skipped (to convert again: delete sim/scenes/simple_cave_01.usd)"
else
  echo "The first Isaac Sim start compiles the render pipelines: ~2 min, peak ~21 GB RAM (later starts ~5 GB)."
  echo "With 32 GB of RAM, close memory-hungry programs such as browsers first."
  docker exec isaac_explore /isaac-sim/python.sh /workspace/isaac_explore/sim/sdf_to_usd.py \
    /workspace/isaac_explore/assets/subt/worlds/simple_cave_01.sdf ||
    fail "scene conversion failed. If it was killed for lack of memory (exit code 137), close other programs and run ./setup.sh again: what was compiled stays cached"
  docker exec isaac_explore chown -R "$(id -u):$(id -g)" /isaac_explore/sim/scenes   # written as root in the container
fi

step 7 "Build the ROS 2 workspace"
./container.sh build

echo
echo "Installation complete. Run:"
echo "  terminal 1: ./sim.sh          (wait for \"[go2_sim] running\")"
echo "  terminal 2: ./explore.sh"
echo "  stop:       ./stop_all.sh"
