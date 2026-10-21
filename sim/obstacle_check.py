"""How many /terrain_map obstacle points (intensity = height above ground) are near the robot?
python3 obstacle_check.py [radius=1.5] [threshold=0.15]"""
import sys, math, numpy as np, rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
r = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
thr = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
rclpy.init(); n = rclpy.create_node("obstacle_check"); st = {}
n.create_subscription(Odometry, "/state_estimation", lambda m: st.__setitem__("p", m.pose.pose.position), 5)
def tm(m):
    if "p" not in st or "done" in st:
        return
    a = pc2.read_points(m, field_names=("x", "y", "z", "intensity"))
    xy = np.stack([a["x"] - st["p"].x, a["y"] - st["p"].y], 1)
    d = np.linalg.norm(xy, axis=1); near = d < r
    obs = near & (a["intensity"] > thr)
    ang = np.degrees(np.arctan2(xy[obs, 1], xy[obs, 0]))
    print(f"[obs] robot(map)=({st['p'].x:.2f},{st['p'].y:.2f}) terrain pts within {r} m: {near.sum()}, "
          f"obstacles (>{thr} m): {obs.sum()}, max height {a['intensity'][near].max() if near.any() else 0:.2f}")
    if obs.any():
        h, e = np.histogram(ang, bins=8, range=(-180, 180))
        print("[obs] obstacle bearing histogram (map frame, deg):", {f"{int(e[i])}..{int(e[i+1])}": int(h[i]) for i in range(8) if h[i]})
    st["done"] = 1
n.create_subscription(PointCloud2, "/terrain_map", tm, 2)
while "done" not in st: rclpy.spin_once(n, timeout_sec=1.0)
