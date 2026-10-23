#!/bin/bash
# How steep can the walking policy climb and descend? One headless sim per lane of scripts/make_ramps.py's scene.
#   ros1_ws/ramp_test.sh [--robot anymal_d] [angles: 10 15 20 25 30 35]     results: logs/ramp_test.txt
set -u
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd)
ROBOT=anymal_d; [ "${1:-}" = --robot ] && { ROBOT=$2; shift 2; }
ANGLES=${*:-10 15 20 25 30 35}
C="docker exec isaac_explore"; IN=/isaac_explore/ros1_ws
"$ROOT/container.sh" start && "$ROOT/stop_all.sh" >/dev/null
[ -f "$HERE/scenes/ramps.usd" ] || { mkdir -p "$HERE/scenes"
  $C bash -c "cd /workspace/isaaclab && ./isaaclab.sh -p $IN/scripts/make_ramps.py $IN/scenes/ramps.usd" | grep DONE; }
SPAWN_Z=$(python3 -c "import yaml; print(yaml.safe_load(open('$ROOT/config/robots.yaml'))['$ROBOT']['spawn_z'])")
for a in $ANGLES; do
  lane=$(python3 -c "print([10, 15, 20, 25, 30, 35].index($a) * 10)")
  $C bash -c "source /opt/ros/humble/setup.bash; cd /workspace/isaaclab; PYTHONUNBUFFERED=1 ./isaaclab.sh -p \
    /workspace/isaac_explore/sim/go2_sim.py --headless --robot $ROBOT --scene $IN/scenes/ramps.usd \
    --spawn -4 $lane $SPAWN_Z > /tmp/ramp_sim.log 2>&1" &
  until $C grep -aqE "\[go2_sim\] running|Traceback" /tmp/ramp_sim.log 2>/dev/null; do sleep 3; done
  $C bash -c "source /isaac_explore/docker/algo_env.sh; python3 $IN/scripts/ramp_test.py $a" | tee -a "$ROOT/logs/ramp_test.txt"
  $C grep -a "\[go2_sim\] stuck\|fallen" /tmp/ramp_sim.log | sed 's/^/      /' | cut -c1-120 | tee -a "$ROOT/logs/ramp_test.txt"
  "$ROOT/stop_all.sh" >/dev/null; wait
done
