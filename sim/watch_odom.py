"""Print an odometry topic's position every ~1 s of sim time: python3 watch_odom.py topic seconds"""
import sys, rclpy
from nav_msgs.msg import Odometry
topic, dur = sys.argv[1], float(sys.argv[2])
rclpy.init(); n = rclpy.create_node("watch"); st = {"last": -1, "t0": None}
def cb(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    st["t0"] = st["t0"] or t
    if t - st["last"] >= 1.0:
        p, v = m.pose.pose.position, m.twist.twist.linear
        print(f"t={t - st['t0']:5.1f} pos=({p.x:8.3f},{p.y:8.3f},{p.z:8.3f}) vel=({v.x:6.2f},{v.y:6.2f},{v.z:6.2f})", flush=True); st["last"] = t
n.create_subscription(Odometry, topic, cb, 50)
while st["t0"] is None or st["last"] - st["t0"] < dur: rclpy.spin_once(n, timeout_sec=1.0)
