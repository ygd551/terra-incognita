#!/bin/bash
# Set up leggedrobotics/terrain-generator (MIT) for isaac_explore, reproducibly:
# pinned commit + our patch, in a venv on top of an existing Python that has torch and open3d (left unmodified).
#   [PY=/path/to/python-with-torch-and-open3d] terrain_gen/setup.sh      (default: python3)
set -e
cd "$(dirname "$0")"
COMMIT=7be9d9a6ebbed7dc669486426184a4694bfd0bb6
if [ ! -d terrain-generator/.git ]; then
  git init -q terrain-generator && git -C terrain-generator remote add origin https://github.com/leggedrobotics/terrain-generator.git
  git -C terrain-generator fetch -q --depth 1 origin $COMMIT && git -C terrain-generator checkout -q FETCH_HEAD
  git -C terrain-generator apply ../patches/boolean_engine_manifold.patch   # no blender here: use manifold3d
fi
if [ ! -x .venv/bin/python ]; then
  PY=${PY:-python3}
  "$PY" -c "import torch, open3d" || { echo "$PY needs torch and open3d: set PY=/path/to/python" >&2; exit 1; }
  "$PY" -m venv --system-site-packages .venv
  # pinning numpy keeps pip from touching the base environment's numpy
  .venv/bin/pip install -c constraints.txt -i https://mirrors.aliyun.com/pypi/simple \
    trimesh networkx alive-progress ray rtree mapbox-earcut manifold3d "git+https://github.com/pvigier/perlin-numpy"
  .venv/bin/pip install --no-deps -e terrain-generator
fi
echo "ready: .venv/bin/python generate.py --cfg indoor|overhanging|overhanging_floor --enable_sdf --mesh_dir out/NAME"
