"""Check the simulator's sensor output for 10 s (wall time), read-only: rates of /imu/data, /lidar/points,
/ground_truth/odom and /clock (sim time per wall time), the last IMU reading and ground-truth pose, and the lidar
geometry (beams, elevation and azimuth range, points per scan, range, scan duration, and how many 1 x 1 deg cells the
last scan hits alone and with the one before: a spinning lidar repeats its pattern, a Livox rosette adds new cells).
Works for every robot of sim.sh.
  inside the container:  source /isaac_explore/docker/algo_env.sh && python3 /isaac_explore/sim/check_sensors.py
"""
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2

rclpy.init()
node = rclpy.create_node("check_sensors")
count, last, prev, clock = {}, {}, {}, []


def keep(name):
    def cb(m):
        count[name] = count.get(name, 0) + 1
        prev[name], last[name] = last.get(name), m
    return cb


for name, typ in (("/imu/data", Imu), ("/lidar/points", PointCloud2), ("/ground_truth/odom", Odometry)):
    node.create_subscription(typ, name, keep(name), 50)
node.create_subscription(Clock, "/clock", lambda m: clock.append(m.clock.sec + m.clock.nanosec * 1e-9),
                         qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.05)
if len(clock) < 2 or len(last) < 3:
    raise SystemExit(f"no simulation output (got {sorted(last)}): is ./sim.sh running?")

sim = clock[-1] - clock[0]
print(f"sim time {sim:.2f} s in 10 s wall time (x{sim / 10:.2f} real time)")
for name in ("/imu/data", "/lidar/points", "/ground_truth/odom"):
    print(f"  {name:20s} {count[name] / sim:6.1f} Hz (sim time)")
a, w = last["/imu/data"].linear_acceleration, last["/imu/data"].angular_velocity
p = last["/ground_truth/odom"].pose.pose.position
print(f"imu acc ({a.x:.2f}, {a.y:.2f}, {a.z:.2f}) m/s^2 (|g| = {np.linalg.norm([a.x, a.y, a.z]):.2f} at rest), "
      f"gyro ({w.x:.3f}, {w.y:.3f}, {w.z:.3f}) rad/s")
print(f"ground truth position ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("t", "<u4")])


def angles(m):
    """-> points, elevation, azimuth [deg]"""
    pts = np.frombuffer(bytes(m.data), DT)
    return pts, np.degrees(np.arctan2(pts["z"], np.hypot(pts["x"], pts["y"]))), np.degrees(np.arctan2(pts["y"], pts["x"]))


def cells(el, az):
    return set(zip(np.floor(el).astype(int), np.floor(az).astype(int)))


pts, el, az = angles(last["/lidar/points"])
r = np.sqrt(pts["x"] ** 2 + pts["y"] ** 2 + pts["z"] ** 2)
print(f"lidar: {len(pts)} points/scan, ~{len(np.unique(np.round(el, 1)))} beams, elevation {el.min():.1f}..{el.max():.1f}"
      f" deg, azimuth {az.min():.1f}..{az.max():.1f} deg, range {r.min():.2f}..{r.max():.1f} m, "
      f"scan {pts['t'].max() * 1e-9:.3f} s")
_, el0, az0 = angles(prev["/lidar/points"])
c1 = cells(el, az)
print(f"lidar 1x1 deg cells hit: {len(c1)} by the last scan, {len(c1 | cells(el0, az0))} by it and the one before")
