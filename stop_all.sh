#!/bin/bash
# Stop the sim and every algorithm node (the container keeps running). Isaac Sim ignores SIGTERM and launch children can
# outlive `ros2 launch`, so this uses SIGKILL (patterns use [x] so pkill does not match its own command line).
C="docker exec isaac_explore"
$C pkill -9 -f "go2_si[m].py|drone_si[m].py|bench_wal[k].py|train_wal[k].py|make_datase[t].py" 2>/dev/null   # sims, benchmark, training, datasets
$C pkill -INT -f "ros2 laun[c]h" 2>/dev/null; sleep 3
$C pkill -9 -f "/isaac_explore/ros2_ws/instal[l]/|/opt/ros/humble/lib/[t]f2_ros|ros2 laun[c]h" 2>/dev/null
# recorders / probes from sim/ (record_run.py, explore_probe.py, watch_*.py): they outlive an interrupted run_trial.sh
$C pkill -INT -f "/isaac_explore/sim/[a-z_]*\.p[y]" 2>/dev/null
$C chown -R "$(id -u):$(id -g)" /isaac_explore/logs 2>/dev/null   # metrics / trajectories are written as root
echo "stopped: sims=$($C pgrep -c -f 'go2_si[m].py|drone_si[m].py' 2>/dev/null)" \
     "nodes=$($C pgrep -c -f '/isaac_explore/ros2_ws/instal[l]/' 2>/dev/null)"
