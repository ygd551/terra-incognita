"""Periodic snapshots of what TARE and terrain analysis see, for offline diagnosis of an exploration run.
  python3 explore_probe.py OUT_DIR [period_sim_s=10] [max_sim_s=600]
Every period (sim time) writes OUT_DIR/snap_<t>.npz with: pose (/state_estimation_at_scan), last way point,
/terrain_map_ext and TARE's uncovered surfaces / frontiers / viewpoint candidates / selected viewpoints /
exploration path / exploring subspaces. Status changes (exploration_finish) go to OUT_DIR/events.txt."""
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

out = sys.argv[1]
period = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
tmax = float(sys.argv[3]) if len(sys.argv) > 3 else 600.0
os.makedirs(out, exist_ok=True)
rclpy.init()
n = rclpy.create_node("explore_probe", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
last = {}

CLOUDS = {"terrain_ext": "/terrain_map_ext", "uncovered": "/uncovered_cloud",
          "frontier": "/uncovered_frontier_cloud", "candidates": "/viewpoint_vis_cloud",
          "selected": "/selected_viewpoint_vis_cloud",
          # need ros2_ws/patches/tare_debug_frontier.patch (raw frontiers, occupancy slice at robot height):
          "frontier_raw": "/frontier_cloud", "frontier_clustered": "/filtered_frontier_cloud",
          "occupancy": "/rolling_occupancy_grid_cloud",
          "cells": "/exploring_cell_vis_cloud",  # subspace status: 1 exploring, 2 covered, ... (UNSEEN absent)
          # all local-horizon viewpoints, intensity = bits 1 collision 2 line-of-sight 4 connected 8 visited
          # 16 candidate 32 has terrain height (with the patch; upstream publishes only the candidates there)
          "vp_state": "/viewpoint_in_collision_cloud_"}


def cloud_cb(key):
    def cb(m):
        names = [f.name for f in m.fields]
        cols = ("x", "y", "z", "intensity") if "intensity" in names else ("x", "y", "z")
        a = pc2.read_points(m, field_names=cols, skip_nans=True)
        last[key] = np.stack([a[c] for c in cols], 1).astype(np.float32) if len(a) else np.zeros((0, len(cols)),
                                                                                                np.float32)
    return cb


for k, topic in CLOUDS.items():
    n.create_subscription(PointCloud2, topic, cloud_cb(k), qos_profile_sensor_data if k == "terrain_ext" else 2)
n.create_subscription(Odometry, "/state_estimation_at_scan", lambda m: last.__setitem__(
    "pose", np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])), 5)
n.create_subscription(PointStamped, "/way_point", lambda m: last.__setitem__(
    "waypoint", np.array([m.point.x, m.point.y, m.point.z])), 5)
for k, topic in (("local_path", "/local_path"), ("global_path", "/global_path")):  # TARE does not publish exploration_path
    n.create_subscription(Path, topic, (lambda key: lambda m: last.__setitem__(key, np.array(
        [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses]).reshape(-1, 3)))(k), 2)
n.create_subscription(Marker, "/tare_visualizer/exploring_subspaces", lambda m: last.__setitem__(
    "subspaces", np.array([[p.x, p.y, p.z] for p in m.points]).reshape(-1, 3)), 2)
events = open(os.path.join(out, "events.txt"), "a")


def on_finish(m):
    if last.get("finish") != m.data:
        last["finish"] = m.data
        events.write(f"{n.get_clock().now().nanoseconds * 1e-9:.1f} exploration_finish={m.data}\n")
        events.flush()


n.create_subscription(Bool, "/exploration_finish", on_finish, 2)

t_next = None
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)
    t = n.get_clock().now().nanoseconds * 1e-9
    if t <= 0 or "pose" not in last:
        continue
    t_next = t_next or t
    if t >= t_next:
        np.savez_compressed(os.path.join(out, f"snap_{t:07.1f}.npz"), t=t,
                            **{k: v for k, v in last.items() if isinstance(v, np.ndarray)})
        print(f"[probe] t={t:.1f} pose={np.round(last['pose'], 1)} wp={np.round(last.get('waypoint', []), 1)} "
              + " ".join(f"{k}={len(last[k])}" for k in ("terrain_ext", "uncovered", "frontier", "candidates",
                                                          "subspaces") if k in last), flush=True)
        t_next += period
    if t > tmax:
        break
