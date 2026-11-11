"""Ideal artifact detector, a stand-in for camera-based detection until the robots have cameras: an artifact of the scene
(sim/scenes/<scene>.artifacts.yaml: true positions, by sim/sdf_to_usd.py) counts as detected when lidar points land on
it (so it is in the line of sight) within RANGE of the lidar. It is reported where the robot's own pose estimate puts it:
its true position relative to the robot, mapped through /bievr_lio/odom, so localization error shows as report error,
as for a real detector. Publishes /artifacts/markers (frame map; RViz display "Artifacts") and logs each detection with
its report error (DARPA SubT scored a report of the right type within 5 m).
  python3 artifacts.py --robot anymal_d|go2|rmf_obelix_ouster64|...      (explore.launch.py starts it; scene: logs/.scene)
  python3 artifacts.py --selftest                           (offline check of the report transform)"""
import sys

import numpy as np

RANGE = 10.0     # m, lidar to artifact: the reach of the detector (knob; a camera detector's depends on size and light)
RADIUS = 0.5     # m, around the artifact centre (0.3 m above its origin) that lidar points must hit
MIN_POINTS = 3
ROOT = "/isaac_explore"
COLORS = {"rescue_randy": (1, .1, .1), "backpack": (1, .5, 0), "phone": (0, 1, 1), "drill": (1, 1, 0),
          "extinguisher": (1, 0, 1), "gas": (0, 1, 0), "vent": (1, 1, 1), "helmet": (.2, .4, 1), "rope": (.6, .4, .2),
          "cube": (.6, .2, 1)}


def rot(q):
    """Quaternion (x, y, z, w) -> rotation matrix."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def report(p_true, pg, Rg, pe, Re):
    """True world position -> where the robot reports it: relative to the true pose (pg, Rg), placed by the estimate."""
    return Re @ (Rg.T @ (p_true - pg)) + pe


def selftest():
    Rg = rot([0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])  # yaw 90 deg
    pg, a = np.array([1.0, 2.0, 0.0]), np.array([1.0, 5.0, 0.5])  # artifact 3 m ahead of the robot
    assert np.allclose(report(a, pg, Rg, pg, Rg), a)  # perfect localization: reported at the true position
    pe = np.array([10.0, 0.0, 0.0])  # estimate in another frame, heading +x (yaw 0): 3 m ahead of it
    assert np.allclose(report(a, pg, Rg, pe, np.eye(3)), [13.0, 0.0, 0.5])
    print("selftest ok")


def main():
    from collections import deque

    import rclpy
    import yaml
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    from visualization_msgs.msg import Marker, MarkerArray

    robot = sys.argv[sys.argv.index("--robot") + 1] if "--robot" in sys.argv else "go2"
    lidar_pos = np.array(yaml.safe_load(open(f"{ROOT}/config/robots.yaml"))[robot]["lidar_pos"])  # mount, no rotation
    scene = open(f"{ROOT}/logs/.scene").read().strip()   # written by sim.sh
    try:
        arts = yaml.safe_load(open(f"{ROOT}/sim/scenes/{scene}.artifacts.yaml")) or []
    except FileNotFoundError:
        arts = []
    if not arts:
        print(f"[artifacts] no artifacts in sim/scenes/{scene}.artifacts.yaml (not a SubT scene, or converted before "
              "v3.0.0): nothing to detect", flush=True)
        return
    for a in arts:
        a["pos"] = np.array(a["pos"], float)
        a["c"] = a["pos"] + [0.0, 0.0, 0.3]
    print(f"[artifacts] {scene}: {len(arts)} artifacts; detected when lidar points hit one within {RANGE:.0f} m",
          flush=True)

    rclpy.init()
    node = rclpy.create_node("artifacts")
    pub = node.create_publisher(MarkerArray, "/artifacts/markers", 5)
    gt, est = deque(maxlen=200), deque(maxlen=50)   # (t, p, R)
    st = {"T_mw": None}   # map <- world, from the first matched true / estimated poses (as eval_lio.py aligns)
    found = {}            # name -> (type, reported position)

    def pose(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        return (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array([p.x, p.y, p.z]),
                rot([q.x, q.y, q.z, q.w]))

    def nearest(buf, t):
        return min(buf, key=lambda e: abs(e[0] - t))

    def on_scan(msg):
        if not gt or not est:
            return
        tg, pg, Rg = nearest(gt, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + 0.05)  # mid-scan
        te, pe, Re = nearest(est, tg)
        if abs(te - tg) > 0.15:
            return
        if st["T_mw"] is None:
            st["T_mw"] = (Re @ Rg.T, pe - Re @ Rg.T @ pg)
        lidar = pg + Rg @ lidar_pos
        todo = [a for a in arts if a["name"] not in found and np.linalg.norm(a["c"] - lidar) < RANGE]
        if todo:
            a3 = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            w = (np.stack([a3["x"], a3["y"], a3["z"]], 1).astype(float) + lidar_pos) @ Rg.T + pg   # world frame
            for a in todo:
                if np.count_nonzero(np.linalg.norm(w - a["c"], axis=1) < RADIUS) < MIN_POINTS:
                    continue
                rep = report(a["pos"], pg, Rg, pe, Re)
                err = np.linalg.norm(rep - (st["T_mw"][0] @ a["pos"] + st["T_mw"][1]))
                found[a["name"]] = (a["type"], rep)
                print(f"[artifacts] {len(found)}/{len(arts)} {a['type']} ({a['name']}) at t={tg:.0f} s, "
                      f"{np.linalg.norm(a['c'] - lidar):.1f} m away: reported ({rep[0]:.1f}, {rep[1]:.1f}, "
                      f"{rep[2]:.1f}), error {err:.2f} m{'' if err <= 5 else ' (> 5 m: would not score)'}", flush=True)
        ma = MarkerArray()
        for i, (typ, p) in enumerate(found.values()):
            for k, shape in enumerate((Marker.SPHERE, Marker.TEXT_VIEW_FACING)):
                m = Marker()
                m.header.frame_id, m.header.stamp = "map", msg.header.stamp
                m.ns, m.id, m.type, m.action = "artifacts", 2 * i + k, shape, Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p[0]), float(p[1]), float(p[2]) + k
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.6
                m.color.r, m.color.g, m.color.b = map(float, COLORS.get(typ, (1, 1, 1)))
                m.color.a = 1.0
                m.text = typ
                ma.markers.append(m)
        if ma.markers:
            pub.publish(ma)

    node.create_subscription(Odometry, "/ground_truth/odom", lambda m: gt.append(pose(m)), 50)
    node.create_subscription(Odometry, "/bievr_lio/odom", lambda m: est.append(pose(m)), 50)
    node.create_subscription(PointCloud2, "/lidar/points", on_scan, 5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(f"[artifacts] found {len(found)}/{len(arts)}: {', '.join(sorted(found)) or '-'}", flush=True)


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else main()
