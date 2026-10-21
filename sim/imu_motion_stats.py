"""IMU statistics per motion segment: python3 imu_motion_stats.py "vx,wz,s" ... (drives the robot itself)."""
import sys, numpy as np, rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
segs = [tuple(map(float, a.split(","))) for a in sys.argv[1:]]
rclpy.init(); n = rclpy.create_node("imu_motion_stats"); pub = n.create_publisher(Twist, "/cmd_vel", 10); buf = []
n.create_subscription(Imu, "/imu/data", lambda m: buf.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)), 1000)
while not buf: rclpy.spin_once(n, timeout_sec=0.5)
for vx, wz, dur in segs:
    c = Twist(); c.linear.x, c.angular.z = vx, wz; t0 = buf[-1][0]; buf.clear()
    while not buf or buf[-1][0] - t0 < dur:
        pub.publish(c); rclpy.spin_once(n, timeout_sec=0.01)
    a = np.array(buf)[len(buf) // 5:]  # skip the transition
    acc, gyr = a[:, 1:4], a[:, 4:7]
    print(f"[imu] vx={vx:+.1f} wz={wz:+.1f}: |acc| mean {np.linalg.norm(acc,axis=1).mean():5.2f} std {np.linalg.norm(acc,axis=1).std():5.2f} "
          f"max {np.linalg.norm(acc,axis=1).max():6.1f} m/s2 | gyro std xyz {np.round(gyr.std(0),2)} max|w| {np.abs(gyr).max():.1f} rad/s", flush=True)
pub.publish(Twist())
