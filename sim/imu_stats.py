"""Mean/std of /imu/data over N messages: python3 imu_stats.py [N]"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import Imu
N = int(sys.argv[1]) if len(sys.argv) > 1 else 400
rclpy.init(); n = rclpy.create_node("imu_stats"); a, w, t = [], [], []
def cb(m):
    a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    w.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]); t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
n.create_subscription(Imu, "/imu/data", cb, 500)
while len(a) < N: rclpy.spin_once(n, timeout_sec=1.0)
a, w, t = map(np.array, (a, w, t))
np.set_printoptions(precision=3, suppress=True)
print("acc mean", a.mean(0), "std", a.std(0), "| |acc| mean", np.linalg.norm(a, axis=1).mean())
print("gyr mean", w.mean(0), "std", w.std(0)); print("dt mean", np.diff(t).mean(), "min", np.diff(t).min(), "max", np.diff(t).max())
print("first 5 acc", a[:5])
