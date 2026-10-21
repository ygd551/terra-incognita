"""Drive test: send vx for N seconds of SIM time, report ground-truth displacement.
python3 drive_test.py [vx] [vy] [wz] [sim_seconds]"""
import sys, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
vx, vy, wz, dur = (list(map(float, sys.argv[1:5])) + [0.5, 0.0, 0.0, 10.0][len(sys.argv) - 1:])[:4]
rclpy.init(); n = rclpy.create_node("drive_test"); pub = n.create_publisher(Twist, "/cmd_vel", 10); st = {}
def on_gt(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    p = m.pose.pose.position
    st.setdefault("t0", t); st.setdefault("p0", (p.x, p.y, p.z)); st["t"], st["p"] = t, (p.x, p.y, p.z)
n.create_subscription(Odometry, "/ground_truth/odom", on_gt, 10)
cmd = Twist(); cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
while "t" not in st or st["t"] - st["t0"] < dur:
    pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.05)
pub.publish(Twist())
p0, p = st["p0"], st["p"]
print(f"[drive] {st['t']-st['t0']:.1f}s sim: start=({p0[0]:.2f},{p0[1]:.2f},{p0[2]:.2f}) end=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}) "
      f"dx={p[0]-p0[0]:.2f} dy={p[1]-p0[1]:.2f}")
