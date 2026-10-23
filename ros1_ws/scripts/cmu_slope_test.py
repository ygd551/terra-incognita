#!/usr/bin/env python3
"""At which slope does the CMU stack call the slope itself an obstacle? Feeds the real terrainAnalysis node (parameters
of launch/cmu/terrain_analysis.launch) a synthetic ramp: flat to x = 1 m, then rising at angle a (5 cm grid, 1 cm
noise, |y| < 2 m), and reports the share of ramp points (x 2..4 m) whose height above the estimated ground exceeds the
local planner's obstacle threshold (config/robots.yaml obstacle_height: anymal_d 0.2, go2 0.15).

  inside the isaac_explore container:  ROS_DOMAIN_ID=77 python3 cmu_slope_test.py [angles...]
(its own ROS domain: no clash with a running stack; one terrainAnalysis per angle, so no points carry over)"""
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

PARAMS = dict(scanVoxelSize=0.05, decayTime=2.0, noDecayDis=4.0, clearingDis=8.0, useSorting=False, quantileZ=0.25,
              considerDrop=True, limitGroundLift=False, maxGroundLift=0.15, clearDyObs=True, minDyObsDis=0.3,
              minDyObsAngle=0.0, minDyObsRelZ=-0.5, absDyObsRelZThre=0.2, minDyObsVFOV=-16.0, maxDyObsVFOV=16.0,
              minDyObsPointNum=1, noDataObstacle=False, noDataBlockSkipNum=0, minBlockPointNum=10, vehicleHeight=1.0,
              voxelPointUpdateThre=100, voxelTimeUpdateThre=2.0, minRelZ=-2.5, maxRelZ=1.0, disRatioZ=0.2)
BASE_Z = 0.55   # robot base above the flat ground


def ramp(angle_deg, rng):
    x, y = np.meshgrid(np.arange(-1.0, 4.5, 0.05), np.arange(-2.0, 2.0, 0.05))
    x, y = x.ravel(), y.ravel()
    z = np.where(x > 1.0, (x - 1.0) * np.tan(np.radians(angle_deg)), 0.0) + rng.normal(0, 0.01, x.size)
    return np.stack([x, y, z, np.zeros_like(x)], 1).astype(np.float32)


def cloud_msg(pts, stamp):
    m = PointCloud2(height=1, width=len(pts), is_bigendian=False, point_step=16, row_step=16 * len(pts), is_dense=True,
                    fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate(["x", "y", "z", "intensity"])],
                    data=pts.tobytes())
    m.header.stamp, m.header.frame_id = stamp, "map"
    return m


def run(angle, node, rng):
    args = ["ros2", "run", "terrain_analysis", "terrainAnalysis", "--ros-args"]
    for k, v in PARAMS.items():
        args += ["-p", "%s:=%s" % (k, str(v).lower() if isinstance(v, bool) else v)]
    # own process group: `ros2 run` forks the node, terminating only the wrapper leaves the node publishing
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    got = {}
    sub = node.create_subscription(PointCloud2, "/terrain_map", lambda m: got.__setitem__("m", m), 2)
    pub_o = node.create_publisher(Odometry, "/state_estimation", 5)
    pub_c = node.create_publisher(PointCloud2, "/registered_scan", 5)
    t_end = time.time() + 6.0
    while time.time() < t_end:
        stamp = node.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "map", "sensor"
        o.pose.pose.position.z, o.pose.pose.orientation.w = BASE_Z, 1.0
        pub_o.publish(o)
        pub_c.publish(cloud_msg(ramp(angle, rng), stamp))
        end = time.time() + 0.2
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
    os.killpg(proc.pid, signal.SIGTERM)
    proc.wait()
    node.destroy_subscription(sub)
    node.destroy_publisher(pub_o)
    node.destroy_publisher(pub_c)
    m = got.get("m")
    if m is None:
        return None
    off = {f.name: f.offset for f in m.fields}   # PCL PointXYZI: intensity at 16 (12 is padding = 1.0)
    a = np.frombuffer(m.data, dtype=np.dtype({"names": ["x", "y", "z", "i"], "formats": ["<f4"] * 4,
                                              "offsets": [off["x"], off["y"], off["z"], off["intensity"]],
                                              "itemsize": m.point_step}), count=m.width * m.height)
    on = (a["x"] > 2.0) & (a["x"] < 4.0) & (np.abs(a["y"]) < 1.0)
    h = a["i"][on]
    return len(h), (h > 0.15).mean() if len(h) else 0, (h > 0.2).mean() if len(h) else 0, h.max() if len(h) else 0


def main():
    angles = [float(a) for a in sys.argv[1:]] or [5, 10, 12.5, 15, 17.5, 20, 22.5, 25, 30, 35]
    rclpy.init()
    node = rclpy.create_node("cmu_slope_test")
    rng = np.random.default_rng(0)
    print("slope  ramp pts  >0.15 m (go2)  >0.20 m (anymal_d)  max height")
    for ang in angles:
        r = run(ang, node, rng)
        print("%5.1f  no terrain_map" % ang if r is None else "%5.1f  %8d  %12.0f%%  %17.0f%%  %9.2f" % (
            ang, r[0], 100 * r[1], 100 * r[2], r[3]), flush=True)


if __name__ == "__main__":
    main()
