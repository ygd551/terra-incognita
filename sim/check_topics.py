"""Sanity check of the sim's ROS 2 output (run with system python3 after sourcing ROS)."""
import numpy as np, rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
rclpy.init(); n = rclpy.create_node("check"); got = {}
def pc(m):
    if "pc" not in got:
        s = pc2.read_points(m, field_names=("x", "y", "z", "t"))
        a = np.stack([s["x"], s["y"], s["z"], s["t"].astype(np.float64)], 1)
        r = np.linalg.norm(a[:, :3], axis=1)
        got["pc"] = f"points={len(a)} range[m] min={r.min():.2f} med={np.median(r):.2f} max={r.max():.2f} t[ms] {a[:,3].min()/1e6:.1f}..{a[:,3].max()/1e6:.1f} stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d}"
n.create_subscription(PointCloud2, "/lidar/points", pc, 2)
n.create_subscription(Odometry, "/ground_truth/odom", lambda m: got.setdefault("gt", f"pos=({m.pose.pose.position.x:.2f},{m.pose.pose.position.y:.2f},{m.pose.pose.position.z:.2f})"), 2)
n.create_subscription(Imu, "/imu/data", lambda m: got.setdefault("imu", f"acc=({m.linear_acceleration.x:.2f},{m.linear_acceleration.y:.2f},{m.linear_acceleration.z:.2f})"), 2)
while len(got) < 3: rclpy.spin_once(n, timeout_sec=1.0)
for k, v in got.items(): print(f"[{k}] {v}")
