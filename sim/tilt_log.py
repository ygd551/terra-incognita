"""Log GT position + body tilt every ~1 s sim while driving vx for N s: python3 tilt_log.py vx seconds"""
import sys, math, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
vx, dur = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); n = rclpy.create_node("tilt_log"); pub = n.create_publisher(Twist, "/cmd_vel", 10); s = {"t0": None, "last": -9}
def gt(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; p, q = m.pose.pose.position, m.pose.pose.orientation
    s["t0"] = s["t0"] or t; s["t"] = t
    if t - s["last"] >= 1.0:
        s["last"] = t
        tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))))
        print(f"t={t - s['t0']:5.1f} pos=({p.x:6.2f},{p.y:6.2f},{p.z:5.2f}) tilt={tilt:5.1f} vx_body={m.twist.twist.linear.x:5.2f}", flush=True)
n.create_subscription(Odometry, "/ground_truth/odom", gt, 50)
c = Twist(); c.linear.x = vx
while s["t0"] is None or s["t"] - s["t0"] < dur: pub.publish(c); rclpy.spin_once(n, timeout_sec=0.05)
pub.publish(Twist())
