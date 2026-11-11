"""Compare an odometry topic with the ground truth, live or while a bag plays (ROS 2, sim time), without moving the
robot (sim/eval_lio.py flies a route itself):
  python3 odom_vs_gt.py TOPIC [SIM_SECONDS=120] [OUT.csv]
  python3 odom_vs_gt.py --bag BAG TOPIC [OUT.csv]      offline, from a recorded ROS 1 or ROS 2 bag holding TOPIC and
                                                       /ground_truth/odom (needs the rosbags package)
  e.g. /bievr_lio/odom (BIEVR-LIO), /aft_mapped_to_init (FAST-LIVO2 through ros1_bridge, ros1_ws/fast_livo2.sh)
The estimate's frame is aligned to the world with its first pose matched to the ground truth (full SE3, as both start at
the same body pose). Prints the position error (mean, max, final, drift over the path) and its course every 10 s, the
longest gap between estimates, and writes t, estimate, ground truth, error to OUT.csv."""
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation

TOPIC = sys.argv[1]
DUR = float(sys.argv[2]) if len(sys.argv) > 2 and TOPIC != "--bag" else 120.0
OUT = sys.argv[3] if len(sys.argv) > 3 else None
est, gt = [], []


def pose(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array([p.x, p.y, p.z]),
            Rotation.from_quat([q.x, q.y, q.z, q.w]))


def report(est, gt):
    tg = np.array([g[0] for g in gt])
    pg = np.array([g[1] for g in gt])
    ok = [e for e in est if tg[0] <= e[0] <= tg[-1]]
    if len(ok) < 2:
        return print(f"{TOPIC}: {len(est)} estimates, {len(gt)} ground-truth poses: nothing to compare")
    te = np.array([e[0] for e in ok])
    g_at = np.c_[[np.interp(te, tg, pg[:, k]) for k in range(3)]].T
    i0 = int(np.argmin(np.abs(tg - te[0])))
    R0 = gt[i0][2] * ok[0][2].inv()                      # world <- estimate frame
    t0 = g_at[0] - R0.apply(ok[0][1])
    pe = R0.apply(np.array([e[1] for e in ok])) + t0
    err = np.linalg.norm(pe - g_at, axis=1)
    path = np.linalg.norm(np.diff(g_at, axis=0), axis=1).sum()
    print(f"{TOPIC}: {len(ok)} estimates over {te[-1] - te[0]:.0f} s sim, path {path:.1f} m | error mean {err.mean():.3f} m,"
          f" max {err.max():.3f} m at {te[err.argmax()] - te[0]:.1f} s, final {err[-1]:.3f} m "
          f"({100 * err[-1] / max(path, 1e-6):.2f} % of the path); longest gap {np.diff(te).max():.2f} s")
    for t in np.arange(0.0, te[-1] - te[0], 10.0):
        i = int(np.searchsorted(te - te[0], t))
        print(f"  t+{t:5.0f} s  error {err[i]:7.3f} m")
    if OUT:
        np.savetxt(OUT, np.c_[te, pe, g_at, err], delimiter=",", fmt="%.4f",
                   header="t,est_x,est_y,est_z,gt_x,gt_y,gt_z,err", comments="")


def selftest():
    # an estimate in a frame turned 90 deg and shifted, otherwise perfect: zero error after the alignment
    T = np.linspace(0, 10, 50)
    gtl = [(t, np.array([t, 0.5 * t, 1.0]), Rotation.from_euler("z", 0.1 * t)) for t in T]
    Rf = Rotation.from_euler("z", np.pi / 2)
    estl = [(t, Rf.inv().apply(p - [3, 4, 0]), Rf.inv() * r) for t, p, r in gtl]
    global OUT
    OUT = None
    report(estl, gtl)


def from_bag(bag, topic):
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    out = {topic: [], "/ground_truth/odom": []}
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:   # Humble bags carry no types
        for conn, _, raw in r.messages(connections=[c for c in r.connections if c.topic in out]):
            m = r.deserialize(raw, conn.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            out[conn.topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array([p.x, p.y, p.z]),
                                    Rotation.from_quat([q.x, q.y, q.z, q.w])))
    return [sorted(out[k], key=lambda e: e[0]) for k in (topic, "/ground_truth/odom")]


if __name__ == "__main__":
    if TOPIC == "--selftest":
        selftest()
        sys.exit()
    if TOPIC == "--bag":
        TOPIC, OUT = sys.argv[3], (sys.argv[4] if len(sys.argv) > 4 else None)
        report(*from_bag(sys.argv[2], TOPIC))
        sys.exit()
    rclpy.init()
    n = rclpy.create_node("odom_vs_gt", parameter_overrides=[Parameter("use_sim_time", value=True)])
    n.create_subscription(Odometry, TOPIC, lambda m: est.append(pose(m)), 50)
    n.create_subscription(Odometry, "/ground_truth/odom", lambda m: gt.append(pose(m)), 50)
    while rclpy.ok() and not gt:
        rclpy.spin_once(n, timeout_sec=0.1)
    t_start = gt[0][0]
    while rclpy.ok() and gt[-1][0] - t_start < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
    report(est, gt)
