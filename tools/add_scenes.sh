#!/bin/bash
# Add simulation scenes: download each world (a DARPA SubT world from osrf/subt, or an .sdf already placed in
# assets/subt/worlds/, e.g. the gbp_* ones) with the Fuel models it uses, and convert it to sim/scenes/NAME.usd.
# Already converted scenes are kept. Then: ./sim.sh --scene NAME (start pose: config/scenes.yaml).
#   tools/add_scenes.sh NAME...          e.g. tools/add_scenes.sh simple_cave_02 simple_tunnel_01
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[ $# -gt 0 ] || { sed -n 2,5p "$0"; exit 1; }
python3 "$ROOT/tools/fetch_assets.py" "$@"
"$ROOT/container.sh" start
for w in "$@"; do
  if [ -f "$ROOT/sim/scenes/$w.usd" ]; then echo "keep    $w"; continue; fi
  echo "convert $w"
  out=$(docker exec isaac_explore /isaac-sim/python.sh /workspace/isaac_explore/sim/sdf_to_usd.py \
    "/workspace/isaac_explore/assets/subt/worlds/$w.sdf" 2>&1 | grep -E "^\[(DONE|WARN)\]|Error|Traceback" || true)
  echo "$out"
  # the stage file exists from the start: only [DONE] means it was saved (a killed run leaves a stub)
  grep -q "^\[DONE\]" <<< "$out" || { echo "FAILED  $w" >&2; rm -f "$ROOT/sim/scenes/$w.usd"; }
done
docker exec isaac_explore chown -R "$(id -u):$(id -g)" /isaac_explore/sim/scenes   # written as root in the container
