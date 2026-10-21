"""Ground-truth stand-in for BIEVR-LIO (explore.launch.py lio:=gt): same topics, frames and stamp convention, so the
planning stack can be tested without localization errors. --robot go2|anymal_d selects the lidar mount
(config/robots.yaml); explore.launch.py lio:=gt passes it.
  /ground_truth/odom + /lidar/points -> /bievr_lio/odom (odom -> imu, body-frame twist), TF odom -> imu,
                                         /bievr_lio/points/registered (frame odom, every point at its true position)
Frame odom = the sim's world frame (so map coordinates equal world coordinates). Stamps = scan end, like BIEVR.
Each lidar point is placed with the ground-truth pose interpolated at its own time (scan start + t), so the cloud
has no motion distortion."""
from collections import deque

import sys

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import TransformBroadcaster

ROBOT_NAME = sys.argv[sys.argv.index("--robot") + 1] if "--robot" in sys.argv else "go2"
with open("/isaac_explore/config/robots.yaml") as f:
    LIDAR_POS = np.array(yaml.safe_load(f)[ROBOT_NAME]["lidar_pos"])  # mount in the base frame, no rotation
OUT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
gt = deque(maxlen=200)  # (t, pos[3], quat wxyz[4], lin_vel_b[3], ang_vel_b[3]) at 50 Hz -> 4 s
scans = deque(maxlen=5)  # waiting until ground truth covers the scan end
OUT_FIELDS = [PointField(name=n, offset=OUT_DTYPE.fields[n][1], datatype=PointField.FLOAT32, count=1)
              for n in OUT_DTYPE.names]

def t_of(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def interp(ts):
    """Ground-truth position and quaternion (wxyz) at times ts (array)."""
    T = np.array([g[0] for g in gt])
    P = np.array([g[1] for g in gt])
    Q = np.array([g[2] for g in gt])
    i = np.clip(np.searchsorted(T, ts), 1, len(T) - 1)
    a = np.clip((ts - T[i - 1]) / np.maximum(T[i] - T[i - 1], 1e-9), 0.0, 1.0)[:, None]
    q0, q1 = Q[i - 1], Q[i]
    q1 = np.where(np.sum(q0 * q1, 1, keepdims=True) < 0, -q1, q1)  # shortest arc
    q = q0 + a * (q1 - q0)
    return P[i - 1] + a * (P[i] - P[i - 1]), q / np.linalg.norm(q, axis=1, keepdims=True)


def rotate(q, v):
    """Rotate vectors v (N,3) by quaternions q (N,4, wxyz)."""
    w, u = q[:, :1], q[:, 1:]
    c = np.cross(u, v)
    return v + 2 * w * c + 2 * np.cross(u, c)


def process(msg):
    a = pc2.read_points(msg, field_names=("x", "y", "z", "t"), skip_nans=True)
    t0 = t_of(msg.header.stamp)
    ts = t0 + a["t"].astype(np.float64) * 1e-9
    t_end = ts.max() if len(ts) else t0
    p_base = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64) + LIDAR_POS
    pos, q = interp(ts)
    out = np.zeros(len(p_base), OUT_DTYPE)
    w = rotate(q, p_base) + pos
    out["x"], out["y"], out["z"] = w[:, 0], w[:, 1], w[:, 2]
    ns = int(round(t_end * 1e9))
    stamp = TimeMsg(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
    cloud = PointCloud2(height=1, width=len(out), fields=OUT_FIELDS, is_bigendian=False,
                        point_step=OUT_DTYPE.itemsize, row_step=OUT_DTYPE.itemsize * len(out), is_dense=True,
                        data=out.tobytes())
    cloud.header.stamp, cloud.header.frame_id = stamp, "odom"

    pe, qe = interp(np.array([t_end]))
    near = min(gt, key=lambda g: abs(g[0] - t_end))
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "imu"
    pp, oo = o.pose.pose.position, o.pose.pose.orientation
    pp.x, pp.y, pp.z = pe[0]
    oo.w, oo.x, oo.y, oo.z = qe[0]
    (o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z) = near[3]
    (o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z) = near[4]
    tr = TransformStamped()
    tr.header, tr.child_frame_id = o.header, "imu"
    tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = pe[0]
    r = tr.transform.rotation
    r.w, r.x, r.y, r.z = qe[0]
    pub_cloud.publish(cloud)
    pub_odom.publish(o)
    tf.sendTransform(tr)


def flush():
    # a scan is published by the sim as soon as its revolution ends, ground truth at the end of the policy step
    # (<= 20 ms later): process a scan once ground truth reaches its end (or when scans pile up: extrapolate)
    while scans and gt:
        msg, t_end = scans[0]
        if gt[-1][0] < t_end and len(scans) < 3:
            return
        scans.popleft()
        process(msg)


def on_gt(m):
    p, q, v, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, m.twist.twist.angular
    gt.append((t_of(m.header.stamp), [p.x, p.y, p.z], [q.w, q.x, q.y, q.z], (v.x, v.y, v.z), (w.x, w.y, w.z)))
    flush()


def on_scan(m):
    a = pc2.read_points(m, field_names=("t",))
    scans.append((m, t_of(m.header.stamp) + (int(a["t"].max()) * 1e-9 if len(a) else 0.0)))
    flush()


if "--selftest" in sys.argv:  # quaternion helpers: 90 deg yaw, half-way interpolation
    gt = deque([(0.0, [0, 0, 0], [1, 0, 0, 0], 0, 0), (1.0, [2, 0, 0], [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)], 0, 0)])
    p, q = interp(np.array([0.5, 1.0]))
    assert np.allclose(p[0], [1, 0, 0]) and np.allclose(rotate(q[1:], np.array([[1.0, 0, 0]])), [[0, 1, 0]], atol=1e-9)
    assert np.allclose(rotate(q[:1], np.array([[1.0, 0, 0]])), [[np.cos(np.pi / 8) ** 2 - np.sin(np.pi / 8) ** 2,
                                                                 2 * np.sin(np.pi / 8) * np.cos(np.pi / 8), 0]])
    print("selftest ok")
    sys.exit(0)

rclpy.init()
node = rclpy.create_node("gt_odom", parameter_overrides=[Parameter("use_sim_time", value=True)])
pub_odom = node.create_publisher(Odometry, "/bievr_lio/odom", 10)
pub_cloud = node.create_publisher(PointCloud2, "/bievr_lio/points/registered", 5)
tf = TransformBroadcaster(node)
node.create_subscription(Odometry, "/ground_truth/odom", on_gt, 50)
node.create_subscription(PointCloud2, "/lidar/points", on_scan, 5)
node.get_logger().info("ground-truth odometry: /ground_truth/odom + /lidar/points -> /bievr_lio/*")
rclpy.spin(node)
