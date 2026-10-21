"""Snapshot of TARE's internal state (run while the stack is up): robot pose, uncovered surfaces/frontiers,
viewpoint candidates, exploring subspaces, and where the uncovered points are relative to the robot."""
import numpy as np, rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
rclpy.init(); n = rclpy.create_node("tare_state"); got = {}
def cloud(name):
    def cb(m):
        if name not in got:
            a = pc2.read_points(m, field_names=("x", "y", "z"))
            got[name] = np.stack([a["x"], a["y"], a["z"]], 1) if len(a) else np.zeros((0, 3))
    return cb
for name, topic in (("uncovered", "/uncovered_cloud"), ("frontier", "/uncovered_frontier_cloud"),
                    ("candidates", "/viewpoint_vis_cloud"), ("selected", "/selected_viewpoint_vis_cloud"),
                    ("explored", "/explored_areas")):
    n.create_subscription(PointCloud2, topic, cloud(name), 2)
n.create_subscription(Odometry, "/state_estimation", lambda m: got.setdefault("pose", m.pose.pose.position), 2)
n.create_subscription(Marker, "/tare_visualizer/exploring_subspaces", lambda m: got.setdefault("subspaces", len(m.points)), 2)
t0 = n.get_clock().now()
while len(got) < 8 and (n.get_clock().now() - t0).nanoseconds < 20e9:
    rclpy.spin_once(n, timeout_sec=0.5)
p = got.get("pose")
if p: print(f"[tare] robot (map) = ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})")
for k in ("uncovered", "frontier", "candidates", "selected", "explored"):
    a = got.get(k)
    if a is None: print(f"[tare] {k:10s}: (no message)"); continue
    msg = f"[tare] {k:10s}: {len(a):6d} pts"
    if len(a) and p is not None:
        d = np.linalg.norm(a[:, :2] - [p.x, p.y], axis=1)
        msg += f", dist from robot min {d.min():.1f} med {np.median(d):.1f} max {d.max():.1f} m, bbox x {a[:,0].min():.1f}..{a[:,0].max():.1f} y {a[:,1].min():.1f}..{a[:,1].max():.1f} z {a[:,2].min():.1f}..{a[:,2].max():.1f}"
    print(msg)
print(f"[tare] exploring subspaces marker points: {got.get('subspaces', '(no message)')}")
