"""Check which frame /lidar/points is really in: grab two scans while the robot turns in place, put both into the
world with the ground-truth base pose, once assuming points are in the lidar frame (base + LIDAR_POS) and once
assuming base frame, and compare how well the two scans overlap. The right hypothesis gives ~cm residuals.
python3 check_lidar_frame.py   (robot must be turning, e.g. drive_test.py 0 0 0.6 10 in parallel)"""
import numpy as np, rclpy
from nav_msgs.msg import Odometry
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
LIDAR_POS = np.array([0.10, 0.0, 0.15])
rclpy.init(); n = rclpy.create_node("check_lidar_frame"); gt, scans = [], []
n.create_subscription(Odometry, "/ground_truth/odom", gt.append, 200)
def on_pc(m):
    a = pc2.read_points(m, field_names=("x", "y", "z", "t"))
    scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.stack([a["x"], a["y"], a["z"]], 1), a["t"] * 1e-9))
n.create_subscription(PointCloud2, "/lidar/points", on_pc, 5)
def pose_at(t):
    m = min(gt, key=lambda g: abs(g.header.stamp.sec + g.header.stamp.nanosec * 1e-9 - t))
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return R.from_quat([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])
def to_world(scan, off):
    t0, pts, dt = scan
    out = np.empty_like(pts)
    for tt in np.unique(dt):  # per-slice pose (rolling scan)
        sel = dt == tt; rot, pos = pose_at(t0 + tt)
        out[sel] = rot.apply(pts[sel] + off) + pos
    return out
while len(scans) < 12: rclpy.spin_once(n, timeout_sec=1.0)
a, b = scans[1], scans[-1]
yaw = lambda s: pose_at(s[0])[0].as_euler("xyz")[2]
print(f"[frame] scans {b[0]-a[0]:.1f} s apart, yaw change {np.degrees(yaw(b)-yaw(a)):.1f} deg")
for name, off in (("lidar frame (base + LIDAR_POS)", LIDAR_POS), ("base frame", np.zeros(3))):
    wa, wb = to_world(a, off), to_world(b, off)
    d, _ = cKDTree(wa).query(wb[np.random.default_rng(0).choice(len(wb), 3000, replace=False)])
    print(f"[frame] assuming {name:32s}: median NN residual {np.median(d)*100:.1f} cm, 90% {np.percentile(d,90)*100:.1f} cm")
