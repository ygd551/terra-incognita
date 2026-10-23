#!/bin/bash
# Fetch the third-party sources at the commits pinned in src.repos (they are not stored in this repo),
# then re-apply our local tweaks. Safe to re-run: existing checkouts are left alone.
set -e
cd "$(dirname "$0")" && mkdir -p src
python3 - <<'PY' | while read -r name url ver; do
import re
txt = open("src.repos").read()
for name, url, ver in re.findall(r"^  (\S+):\n    type: git\n    url: (\S+)\n    version: (\S+)", txt, re.M):
    print(name, url, ver)
PY
  if [ -d "src/$name/.git" ]; then echo "keep  $name"; continue; fi
  echo "fetch $name @ ${ver:0:7}"
  git init -q "src/$name" && git -C "src/$name" remote add origin "$url"
  git -C "src/$name" fetch -q --depth 1 origin "$ver" && git -C "src/$name" checkout -q FETCH_HEAD
done
# packages we do not build: Gazebo-based vehicle/velodyne sims (Isaac replaces them), BIEVR's ROS1 interface,
# elevation mapping's optional plane segmentation (extra CGAL deps)
for p in autonomous_exploration_development_environment/src/vehicle_simulator \
         autonomous_exploration_development_environment/src/velodyne_simulator \
         BIEVR-LIO/interfaces/ros1 elevation_mapping_cupy/plane_segmentation; do
  touch "src/$p/COLCON_IGNORE"
done
# required source patches (patches/*debug* are optional diagnostics, apply by hand:
#   git -C src/tare_planner apply ../../patches/tare_debug_frontier.patch)
for pair in local_planner_ignore_footprint:autonomous_exploration_development_environment \
            tare_direction_change:tare_planner tare_rviz_trajectory:tare_planner; do
  P=patches/${pair%%:*}.patch; A=src/${pair##*:}
  if git -C $A apply --check ../../$P 2>/dev/null; then git -C $A apply ../../$P && echo "patch $P"; else echo "keep  $P (already applied)"; fi
done
echo done
