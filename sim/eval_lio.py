"""Drive (or fly) a route via /cmd_vel and compare LIO odometry with ground truth.
python3 eval_lio.py [odom_topic]   (default /bievr_lio/odom)
Route: list of (vx, vy, vz, wz, sim_seconds); a walk for the legged robots, a 3D flight for the drone (the robot ./sim.sh
was started with, logs/.robot). LIO's odom frame is aligned to GT with the first matched pose (SE3)."""
import sys, numpy as np, rclpy, yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

WALK = [(0.0, 0, 0, 0.0, 2), (0.6, 0, 0, 0.0, 15), (0.5, 0, 0, 0.4, 8), (0.6, 0, 0, 0.0, 15), (0.5, 0, 0, -0.4, 8),
        (0.6, 0, 0, 0.0, 15), (0.0, 0, 0, 0.0, 2)]
# take off, a 3 x 3 m square at 1 m/s with climbs and descents, a turn in place, land-ish: needs ~4 m of free space
FLY = [(0, 0, 0, 0, 2), (0, 0, 0.5, 0, 3), (1.0, 0, 0, 0, 3), (0, 1.0, 0.3, 0, 3), (-1.0, 0, -0.3, 0, 3),
       (0, -1.0, 0, 0, 3), (0, 0, 0, 0.5, 6), (0.5, 0, 0, 0.3, 6), (0, 0, -0.5, 0, 2), (0, 0, 0, 0, 2)]
ROBOT = open("/isaac_explore/logs/.robot").read().strip()
ROUTE = FLY if yaml.safe_load(open("/isaac_explore/config/robots.yaml"))[ROBOT].get("type") == "aerial" else WALK
topic = sys.argv[1] if len(sys.argv) > 1 else "/bievr_lio/odom"
rclpy.init(); n = rclpy.create_node("eval_lio"); pub = n.create_publisher(Twist, "/cmd_vel", 10)
gt, lio = {}, {}
def T(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    M = np.eye(4); M[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix(); M[:3, 3] = [p.x, p.y, p.z]; return M
key = lambda m: round((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) * 100)  # 10 ms bins
n.create_subscription(Odometry, "/ground_truth/odom", lambda m: gt.__setitem__(key(m), T(m)), 50)
n.create_subscription(Odometry, topic, lambda m: lio.__setitem__(key(m), T(m)), 50)
while not gt: rclpy.spin_once(n, timeout_sec=0.5)
for vx, vy, vz, wz, dur in ROUTE:
    t0 = max(gt); c = Twist(); c.linear.x, c.linear.y, c.linear.z, c.angular.z = float(vx), float(vy), float(vz), float(wz)
    while max(gt) - t0 < dur * 100:
        pub.publish(c); rclpy.spin_once(n, timeout_sec=0.02)
pub.publish(Twist())
ks = sorted(set(gt) & set(lio))
assert ks, f"no time-matched poses between GT and {topic}"
A = gt[ks[0]] @ np.linalg.inv(lio[ks[0]])  # odom -> world
err = np.array([np.linalg.norm((A @ lio[k])[:3, 3] - gt[k][:3, 3]) for k in ks])
dist = sum(np.linalg.norm(gt[b][:3, 3] - gt[a][:3, 3]) for a, b in zip(ks, ks[1:]))
end = A @ lio[ks[-1]]; yaw_err = np.degrees(abs(R.from_matrix(end[:3, :3].T @ gt[ks[-1]][:3, :3]).as_rotvec()[2]))
print(f"[eval] {topic}: {len(ks)} matched poses, path {dist:.1f} m | APE mean {err.mean():.3f} m, max {err.max():.3f} m, "
      f"final {err[-1]:.3f} m ({100 * err[-1] / max(dist, 1e-6):.2f}% drift), final yaw err {yaw_err:.2f} deg")
