#!/bin/bash
# Smoke test of scenes: start each headless with its default start pose (config/scenes.yaml) and check that the robot
# stands there: no settling retries, upright, base at the spawn height (not fallen through a missing floor).
#   tools/check_scenes.sh [--robot anymal_d|go2] [NAME...]      (default: every sim/scenes/*.usd)   -> logs/check_scenes.txt
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd); C="docker exec isaac_explore"
ROBOT=anymal_d; [ "${1:-}" = --robot ] && { ROBOT=$2; shift 2; }
SCENES=${*:-$(ls "$ROOT"/sim/scenes/*.usd | xargs -n1 basename | sed 's/\.usd$//')}
"$ROOT/container.sh" start && "$ROOT/stop_all.sh" >/dev/null
for s in $SCENES; do
  $C rm -f /tmp/check_scene.log   # else the wait below can read the previous scene's "running" before the new sim starts
  $C bash -c "source /opt/ros/humble/setup.bash; cd /workspace/isaaclab; PYTHONUNBUFFERED=1 ./isaaclab.sh -p \
    /workspace/isaac_explore/sim/go2_sim.py --headless --robot $ROBOT --scene $s > /tmp/check_scene.log 2>&1" &
  t0=$(date +%s)
  until $C grep -aqE "\[go2_sim\] running|Traceback|Error executing" /tmp/check_scene.log 2>/dev/null; do
    sleep 3; [ $(($(date +%s) - t0)) -gt 600 ] && break
  done
  start=$($C grep -ao "start at ([^)]*) heading [-0-9]* deg" /tmp/check_scene.log | head -1)
  retries=$($C grep -ac "did not stand still" /tmp/check_scene.log)
  if $C grep -aq "\[go2_sim\] running" /tmp/check_scene.log; then
    pose=$($C bash -c "source /isaac_explore/docker/algo_env.sh; timeout 30 python3 - <<'EOF'
import math, time, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('check_scene'); s = []
n.create_subscription(Odometry, '/ground_truth/odom', lambda m: s.append(m.pose.pose), 10)
t = time.time()
while time.time() - t < 6: rclpy.spin_once(n, timeout_sec=0.1)
p, q = s[-1].position, s[-1].orientation
print('%.2f %.0f' % (p.z, math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))))))
EOF")
    z0=$(echo "$start" | sed -E 's/.*, ([-0-9.]+)\) heading.*/\1/')
    read -r z tilt <<< "$pose"
    ok=$(python3 -c "print('OK' if $tilt < 30 and abs($z - ($z0)) < 0.4 and $retries == 0 else 'CHECK')")
    printf "%-28s %-5s %s | now z %s tilt %s deg | settling retries %s\n" "$s" "$ok" "$start" "$z" "$tilt" "$retries"
  else
    printf "%-28s FAIL  %s | no '[go2_sim] running' in %ss: %s\n" "$s" "$start" "$(($(date +%s) - t0))" \
      "$($C grep -aE 'Error|Traceback' /tmp/check_scene.log | tail -1 | cut -c1-100)"
  fi | tee -a "$ROOT/logs/check_scenes.txt"
  "$ROOT/stop_all.sh" >/dev/null; wait
done
