#!/bin/bash
# Fetch GBPlanner2 (ROS 1 Noetic) and its dependencies at the commits pinned in src.repos into src/.
# Safe to re-run: existing checkouts are kept.
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
# not built: GBPlanner's own Gazebo simulation (Isaac Sim replaces it; it would pull in Gazebo); yaml_cpp_catkin
# (nothing depends on it, and it git-clones yaml-cpp at build time); catkin_simple's test packages
for p in gbplanner_ros/planner_gazebo_sim yaml_cpp_catkin catkin_simple/test; do touch "src/$p/CATKIN_IGNORE"; done
# our changes to GBPlanner (README, GBPlanner2): log level WARN (a compile-time macro, upstream ERROR hides why a
# plan failed); two guards against an upstream crash (global planner picking an unreachable frontier, empty path)
P=$PWD/patches/gbplanner_ros.patch
if git -C src/gbplanner_ros apply --reverse --check "$P" 2>/dev/null; then echo "patch already applied"
else git -C src/gbplanner_ros apply "$P" && echo "patched gbplanner_ros"; fi
echo done
