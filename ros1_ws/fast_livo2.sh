#!/bin/bash
# FAST-LIVO2 (LiDAR-inertial-visual odometry with a colored map, hku-mars, GPL-2.0) for the Isaac Sim robot, in the ROS 1
# container, in the background. Normally started by ./explore.sh lio:=fastlivo2, which then localizes the whole stack
# with it (sim/gt_odom.py --pose /aft_mapped_to_init -> /bievr_lio/*).
#   ros1_ws/fast_livo2.sh [start]   ROS 1 master on port 11312 (independent of GBPlanner2's 11311) + FAST-LIVO2 + its
#                                   RViz (the colored map), ros1_bridge for its inputs and its pose
#   ros1_ws/fast_livo2.sh stop      FAST-LIVO2 writes the colored map (src/FAST-LIVO2/Log/pcd/all_raw_points.pcd and
#                                   all_downsampled_points.pcd), then everything stops
# Env: ROBOT, LIDAR (default: what ./sim.sh was started with), RVIZ=false (no RViz window),
#      FAST_LIVO2_IMG=0|1: LIO only although images come (to tell the visual update's effects from the rest) /
#      use the images although none are published yet (a bag about to play)
# Colors need camera images: start the sim without --no_camera (drones have a camera; without images: LIO only).
# Log: docker exec gbplanner_ros1 tail -f /tmp/fast_livo2_launch.log
set -e
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd)
R1="docker exec gbplanner_ros1"
PORT=11312
ENV1="export ROS_MASTER_URI=http://localhost:$PORT && source /ws/devel/setup.bash"

if [ "${1:-}" = stop ]; then
  docker rm -f fastlivo2_bridge fastlivo2_bridge_img >/dev/null 2>&1 || true
  if $R1 pgrep -f "fastlivo_mappin[g]" >/dev/null 2>&1; then
    $R1 pkill -INT -f "fastlivo_mappin[g]" || true   # it writes the map on SIGINT: wait for that
    for i in $(seq 120); do $R1 pgrep -f "fastlivo_mappin[g]" >/dev/null 2>&1 || break; sleep 1; done
  fi
  $R1 pkill -INT -f "fast_livo2_isaac.launc[h]" 2>/dev/null || true; sleep 2
  $R1 pkill -9 -f "fastlivo_mappin[g]|fast_livo2_isaac.launc[h]|rosmaste[r] --core -p $PORT|fast_livo2(_isaac)?.rvi[z]|color_ma[p].py" \
    2>/dev/null || true
  $R1 chown -R "$(id -u):$(id -g)" /ws/src/FAST-LIVO2/Log 2>/dev/null || true
  echo "fast_livo2 stopped; colored map: $(ls "$HERE"/src/FAST-LIVO2/Log/pcd/*.pcd 2>/dev/null | xargs || echo none)"
  exit 0
fi

[ -f "$HERE/devel/.private/fast_livo/lib/fast_livo/fastlivo_mapping" ] || {
  echo "FAST-LIVO2 is not built: ./fetch_src.sh (needs Sophus a621ff and rpg_vikit in THIRDPARTY_ROOT), ./ros1.sh build" >&2
  exit 1; }
ROBOT=${ROBOT:-$(cat "$ROOT/logs/.robot" 2>/dev/null || echo anymal_d)}
LIDAR=${LIDAR:-$(cat "$ROOT/logs/.lidar" 2>/dev/null || echo os1_32)}
# camera images only exist when the sim runs a camera (drones, without --no_camera)
CAM=0
docker exec isaac_explore bash -c "source /isaac_explore/docker/algo_env.sh && timeout 10 ros2 topic list" 2>/dev/null \
  | grep -qx /camera/image_raw && CAM=1
IMG=${FAST_LIVO2_IMG:-$CAM}
CFG=$HERE/logs/fast_livo2_$ROBOT.yaml; mkdir -p "$HERE/logs"
# per-robot values: lidar mount = LiDAR in the IMU (base) frame, no rotation (config/robots.yaml); camera = pinhole of
# sim_common.camera_k looking along +x, ROS optical frame (z forward, x right, y down): p_cam = Rcl p_lidar + Pcl
python3 - "$ROOT" "$ROBOT" "$LIDAR" "$IMG" "$CFG" <<'PY'
import math, sys, yaml
root, robot, lidar, cam_on, out = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4]), sys.argv[5]
R = yaml.safe_load(open(f"{root}/config/robots.yaml"))[robot]
L = yaml.safe_load(open(f"{root}/config/lidars.yaml"))[lidar]
lp = [float(v) for v in R["lidar_pos"]]
c = R.get("camera") or {"pos": [0.0, 0.0, 0.0], "width": 640, "height": 480, "hfov": 90.0}   # unused without images
Rcb = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]          # base (x fwd, y left, z up) -> camera optical
d = [lp[i] - c["pos"][i] for i in range(3)]                          # lidar origin relative to the camera, base axes
f = c["width"] / 2 / math.tan(math.radians(c["hfov"]) / 2)
cfg = {"common": {"img_en": int(bool(cam_on and "camera" in R))},
       "extrin_calib": {"extrinsic_T": lp, "extrinsic_R": [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
                        "Rcl": [v for row in Rcb for v in row],
                        "Pcl": [sum(Rcb[r][k] * d[k] for k in range(3)) for r in range(3)]},
       "preprocess": {"point_filter_num": 1 if L.get("type") == "livox" else 2},   # Avia 24k points/scan, Ouster 55k+
       "laserMapping": {"cam_model": "Pinhole", "cam_width": c["width"], "cam_height": c["height"], "scale": 1.0,
                        "cam_fx": f, "cam_fy": f, "cam_cx": c["width"] / 2, "cam_cy": c["height"] / 2,
                        "cam_d0": 0.0, "cam_d1": 0.0, "cam_d2": 0.0, "cam_d3": 0.0}}
yaml.safe_dump(cfg, open(out, "w"), default_flow_style=None)
PY
"$HERE/ros1.sh" start
"$0" stop >/dev/null
$R1 bash -c "rm -f /ws/src/FAST-LIVO2/Log/pcd/*.pcd"   # this run's map only
docker exec -d gbplanner_ros1 bash -c "$ENV1 && roslaunch -p $PORT /ws/launch/fast_livo2_isaac.launch \
  rviz:=${RVIZ:-true} robot_config:=/ws/logs/fast_livo2_$ROBOT.yaml > /tmp/fast_livo2_launch.log 2>&1"
for i in $(seq 30); do $R1 bash -c "$ENV1 && rosparam get /use_sim_time" >/dev/null 2>&1 && break; sleep 1; done
# its RViz draws scripts/color_map.py's voxel map: RViz keeping every scan itself grew past 6 GB and was killed (OOM)
[ "${RVIZ:-true}" = false ] || docker exec -d gbplanner_ros1 bash -c \
  "$ENV1 && exec python3 -u /ws/scripts/color_map.py > /tmp/color_map.log 2>&1"
# two bridges: the images (1.2 MB at 10 Hz) on their own, as one single-threaded bridge held the lidar scans back
# behind them (0.5-1 s late, "IMU and LiDAR not synced"); the image one in namespace img (its ROS 1 node name and
# its /img/topics parameter), so the two do not collide on the one master
bridge() {   # NAME NAMESPACE YAML
  docker run -d --rm --name "$1" --network host -e ROS_MASTER_URI=http://localhost:$PORT -e ROS_NAMESPACE="$2" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
    -v "$ROOT/config/fastdds.xml":/fastdds.xml:ro -v "$3":/bridge.yaml:ro ros:foxy-ros1-bridge \
    bash -c "source /opt/ros/noetic/setup.bash && rosparam load /bridge.yaml /$2 && \
             source /opt/ros/foxy/setup.bash && exec ros2 run ros1_bridge parameter_bridge" >/dev/null
}
BRIDGE=$HERE/logs/bridge_fast_livo2.yaml
grep -v "/camera/image_raw" "$HERE/config/bridge_fast_livo2.yaml" > "$BRIDGE"
bridge fastlivo2_bridge "" "$BRIDGE"
if [ $CAM = 1 ] || [ $IMG = 1 ]; then   # images seen or forced
  grep -E "^topics:|/camera/image_raw" "$HERE/config/bridge_fast_livo2.yaml" > "$HERE/logs/bridge_fast_livo2_img.yaml"
  bridge fastlivo2_bridge_img img "$HERE/logs/bridge_fast_livo2_img.yaml"
fi
if [ $IMG = 1 ]; then MODE="camera images: colored map"
elif [ $CAM = 1 ]; then MODE="images ignored (FAST_LIVO2_IMG=0): LIO only"
else MODE="NO camera images: LIO only, no colors (sim without --no_camera, robot with a camera)"; fi
echo "FAST-LIVO2 started for $ROBOT with lidar $LIDAR, $MODE (log: docker exec gbplanner_ros1 tail -f /tmp/fast_livo2_launch.log)"
