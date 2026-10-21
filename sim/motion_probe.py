"""Run a sequence of (vx, wz, seconds) commands and print LIO vs ground-truth displacement per segment.
python3 motion_probe.py "vx,wz,s" ["vx,wz,s" ...]"""
import math, sys, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
segs = [tuple(map(float, a.split(","))) for a in sys.argv[1:]]
rclpy.init(); n = rclpy.create_node("motion_probe"); pub = n.create_publisher(Twist, "/cmd_vel", 10); st = {}
def keep(k):
    def cb(m):
        p = m.pose.pose.position; st[k] = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z)
    return cb
n.create_subscription(Odometry, "/ground_truth/odom", keep("gt"), 50)
n.create_subscription(Odometry, "/bievr_lio/odom", keep("lio"), 50)
while "gt" not in st or "lio" not in st: rclpy.spin_once(n, timeout_sec=0.5)
for vx, wz, dur in segs:
    g0, l0 = st["gt"], st["lio"]; c = Twist(); c.linear.x, c.angular.z = vx, wz
    while st["gt"][0] - g0[0] < dur:
        pub.publish(c); rclpy.spin_once(n, timeout_sec=0.02)
    g1, l1 = st["gt"], st["lio"]
    print(f"[seg] vx={vx:+.1f} wz={wz:+.1f} {dur:.0f}s: GT moved {math.dist(g0[1:3], g1[1:3]):5.2f} m (z {g1[3]:.2f}) | "
          f"LIO moved {math.dist(l0[1:3], l1[1:3]):7.2f} m", flush=True)
pub.publish(Twist())
