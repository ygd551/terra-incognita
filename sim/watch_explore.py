"""Exploration monitor: GT position, travelled distance, latest waypoint and cmd every ~5 s sim. python3 watch_explore.py seconds"""
import sys, math, rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60
rclpy.init(); n = rclpy.create_node("watch_explore"); s = {"t0": None, "last": -9, "dist": 0.0, "p": None, "wp": None, "cmd": None}
def gt(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; p = m.pose.pose.position
    s["t0"] = s["t0"] or t
    if s["p"]: s["dist"] += math.dist(s["p"], (p.x, p.y))
    s["p"], s["t"] = (p.x, p.y), t
    if t - s["last"] >= 5:
        s["last"] = t
        print(f"t={t - s['t0']:6.1f}s gt=({p.x:7.2f},{p.y:7.2f},{p.z:5.2f}) travelled={s['dist']:6.1f}m way_point={s['wp']} cmd={s['cmd']}", flush=True)
n.create_subscription(Odometry, "/ground_truth/odom", gt, 50)
n.create_subscription(PointStamped, "/way_point", lambda m: s.__setitem__("wp", f"({m.point.x:.1f},{m.point.y:.1f})"), 5)
n.create_subscription(TwistStamped, "/cmd_vel_stamped", lambda m: s.__setitem__("cmd", f"({m.twist.linear.x:.2f},{m.twist.angular.z:.2f})"), 5)
while s["t0"] is None or s["t"] - s["t0"] < dur: rclpy.spin_once(n, timeout_sec=1.0)
