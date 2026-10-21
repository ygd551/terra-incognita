"""Compare the IMU gyro with the ground-truth yaw rate while turning: python3 check_gyro.py (turn in parallel)."""
import numpy as np, rclpy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
rclpy.init(); n = rclpy.create_node("check_gyro"); imu, gt = [], []
n.create_subscription(Imu, "/imu/data", lambda m: imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
    m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)), 500)
def on_gt(m):
    q = m.pose.pose.orientation
    gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2],
               m.twist.twist.angular.z))
n.create_subscription(Odometry, "/ground_truth/odom", on_gt, 200)
while len(gt) < 150: rclpy.spin_once(n, timeout_sec=1.0)
g = np.array(gt); i = np.array(imu)
yaw = np.unwrap(g[:, 1]); rate_gt = np.diff(yaw) / np.diff(g[:, 0])
print(f"[gyro] GT yaw rate from orientation: mean {rate_gt.mean():+.3f} rad/s | GT twist.angular.z mean {g[:,2].mean():+.3f}")
print(f"[gyro] IMU angular_velocity.z mean {i[:,1].mean():+.3f} rad/s over {len(i)} msgs")
print(f"[gyro] IMU acc mean ({i[:,2].mean():+.2f}, {i[:,3].mean():+.2f}, {i[:,4].mean():+.2f}), |acc| {np.linalg.norm(i[:,2:5],axis=1).mean():.2f}")
