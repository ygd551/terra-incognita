"""Flight test for drone_sim.py: a fixed sequence of velocity commands on /cmd_vel, then per segment what the drone
did (ground truth): displacement against the commanded one, height, largest tilt, yaw turned, drift while hovering.
  inside the container, with ./sim.sh --robot rmf_obelix_ouster64 (or _avia) running and nothing else commanding the drone:
  source /isaac_explore/docker/algo_env.sh && python3 /isaac_explore/sim/drone_test.py [--speed 1.0] [--height 1.0]
The drone takes off from where it is: give it ~2 m of free space ahead, to the left and above (the start area of the
SubT caves has it).
--step: step responses of the flight controller alone (no planner): 5 s forward and back, 3 s left and right, a full
  circle (forward + yaw rate), each followed by a hover. Per segment: the speed along the command (peak, overshoot,
  time to 90 %, settling time to +-10 %, oscillation period), the sideways speed and drift, the overshoot after the stop;
  for the circle the velocity direction against the commanded one (turned into the world with the measured yaw, as a
  flight controller in velocity mode does) and drone_sim's reference yaw against the measured yaw. Needs ~5 m ahead and
  ~4 m to the left at 1 m height: ./sim.sh --robot rmf_obelix_ouster64 --yaw 180 (simple_cave_01) has it.
"""
import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ap = argparse.ArgumentParser()
ap.add_argument("--speed", type=float, default=1.0, help="horizontal speed of the test segments [m/s]")
ap.add_argument("--height", type=float, default=1.0, help="climb at the start [m]")
ap.add_argument("--step", action="store_true", help="step responses instead (see above)")
args = ap.parse_args()
v, h = args.speed, args.height
# (name, seconds, vx, vy, vz, yaw rate): x forward, y left, z up, in the drone's heading frame
SEGMENTS = [("take off", h / 0.5, 0, 0, 0.5, 0), ("hover", 4, 0, 0, 0, 0),
            ("forward", 2 / v, v, 0, 0, 0), ("hover", 4, 0, 0, 0, 0),
            ("left", 2 / v, 0, v, 0, 0), ("hover", 4, 0, 0, 0, 0),
            ("yaw +90 deg", math.pi / 2 / 0.5, 0, 0, 0, 0.5), ("hover", 4, 0, 0, 0, 0),
            ("down", h / 2 / 0.5, 0, 0, -0.5, 0), ("hover", 4, 0, 0, 0, 0)]
WZ = 0.5   # rad/s, the circle's yaw rate (the adapter's YAW_RATE_MAX)
if args.step:
    SEGMENTS = [("take off", h / 0.5, 0, 0, 0.5, 0), ("hover", 4, 0, 0, 0, 0),
                ("forward", 5, v, 0, 0, 0), ("hover", 5, 0, 0, 0, 0), ("back", 5, -v, 0, 0, 0), ("hover", 5, 0, 0, 0, 0),
                ("left", 3, 0, v, 0, 0), ("hover", 5, 0, 0, 0, 0), ("right", 3, 0, -v, 0, 0), ("hover", 5, 0, 0, 0, 0),
                ("forward", 3, v, 0, 0, 0), ("hover", 5, 0, 0, 0, 0),
                ("circle", 2 * math.pi / WZ, v, 0, 0, WZ), ("hover", 5, 0, 0, 0, 0),
                ("down", h / 2 / 0.5, 0, 0, -0.5, 0), ("hover", 3, 0, 0, 0, 0)]

rclpy.init()
node = rclpy.create_node("drone_test")
pub = node.create_publisher(Twist, "/cmd_vel", 10)
log = []   # (t, position, yaw, tilt [deg], world velocity)


def quat_rot(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def on_gt(m):
    p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear   # twist: body frame (sim_common.gt)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    log.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array([p.x, p.y, p.z]), yaw, tilt,
                quat_rot(q.w, q.x, q.y, q.z) @ np.array([v.x, v.y, v.z])))


def step_report(name, seg, nxt, vx, vy, wz):
    """Step-response numbers of one moving segment (seg) and the hover after it (nxt), from the ground truth."""
    t = np.array([x[0] for x in seg]) - seg[0][0]
    vw = np.array([x[4][:2] for x in seg])
    yaw = np.unwrap([x[2] for x in seg])
    vc = math.hypot(vx, vy)
    if wz == 0:
        a = seg[0][2] + math.atan2(vy, vx)                      # commanded direction in the world (heading at start)
        u, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        along, side = vw @ u, vw @ n
        k90 = np.flatnonzero(along >= 0.9 * vc)
        t90 = t[k90[0]] if len(k90) else float("nan")
        out = np.flatnonzero(np.abs(along - vc) > 0.1 * vc)
        settle = t[out[-1]] if len(out) else 0.0
        e = along[k90[0]:] - vc if len(k90) else np.zeros(1)
        zc = t[k90[0]:][np.flatnonzero(np.diff(np.sign(e)) != 0)] if len(k90) else []
        period = 2 * np.mean(np.diff(zc)) if len(zc) > 2 else float("nan")
        pa = np.array([x[1][:2] for x in nxt]) @ u                # position along the command during the hover
        stop = pa.max() - pa[-1] if vc > 0 else 0.0
        drift = (seg[-1][1][:2] - seg[0][1][:2]) @ n
        print(f"  step {name:8s}: peak {along.max():.2f} m/s (overshoot {100 * (along.max() / vc - 1):+.0f} %), "
              f"90 % after {t90:.2f} s, within +-10 % after {settle:.1f} s"
              f"{' (never settled)' if settle > t[-1] - 0.2 else ''}, oscillation period {period:.1f} s; sideways "
              f"speed max {np.abs(side).max():.2f} m/s, drift {drift:+.2f} m; after the stop overshoot {stop:.2f} m",
              flush=True)
    else:
        a = yaw + math.atan2(vy, vx)                             # commanded direction, turned with the measured yaw
        va = np.unwrap(np.arctan2(vw[:, 1], vw[:, 0]))
        moving = np.linalg.norm(vw, axis=1) > 0.5 * vc
        err = np.degrees((va - a + np.pi) % (2 * np.pi) - np.pi)[moving]
        ramp = np.minimum(t, wz / 2.0)                           # drone_sim's command slew: 2 rad/s^2
        yref = yaw[0] + wz * (t - ramp) + 0.5 * 2.0 * ramp ** 2 * np.sign(wz)   # its reference yaw (integrated)
        gap = np.degrees(yref - yaw)
        r = np.linalg.norm(vw, axis=1).mean() / max(abs(np.gradient(yaw, t)).mean(), 1e-6)
        print(f"  step {name:8s}: velocity direction vs commanded: mean {np.abs(err).mean():.0f} deg, max "
              f"{np.abs(err).max():.0f} deg; reference yaw - measured yaw: mean {gap.mean():+.0f} deg, max "
              f"{np.abs(gap).max():.0f} deg; turned {np.degrees(yaw[-1] - yaw[0]):.0f} deg, radius {r:.2f} m "
              f"(commanded {vc / wz:.2f} m), start-to-end {np.linalg.norm(seg[-1][1][:2] - seg[0][1][:2]):.2f} m",
              flush=True)


node.create_subscription(Odometry, "/ground_truth/odom", on_gt, 50)
while not log:
    rclpy.spin_once(node, timeout_sec=0.1)
print(f"{'segment':12s} {'s':>5s}  {'commanded (heading frame)':>27s}  {'flown (heading frame)':>24s}  {'height':>6s}  "
      f"{'max tilt':>8s}  {'yaw':>6s}")
segs = []
for name, dur, vx, vy, vz, wz in SEGMENTS:
    t0, p0, yaw0 = log[-1][:3]
    cmd = Twist()
    cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = float(vx), float(vy), float(vz), float(wz)
    while log[-1][0] - t0 < dur:
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.02)
    seg = [x for x in log if x[0] >= t0]
    segs.append((name, seg, vx, vy, wz))
    t1, p1, yaw1 = log[-1][:3]
    c, s = math.cos(yaw0), math.sin(yaw0)
    d = p1 - p0
    flown = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
    turned = math.degrees((yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    print(f"{name:12s} {t1 - t0:5.1f}  ({vx * dur:5.2f}, {vy * dur:5.2f}, {vz * dur:5.2f}) m  "
          f"({flown[0]:5.2f}, {flown[1]:5.2f}, {flown[2]:5.2f}) m  {p1[2]:6.2f}  {max(x[3] for x in seg):6.1f} deg  "
          f"{turned:+5.0f} deg", flush=True)
pub.publish(Twist())
if args.step:
    print("step responses (ground truth):")
    for i, (name, seg, vx, vy, wz) in enumerate(segs):
        if (vx or vy) and i + 1 < len(segs):
            step_report(name, seg, segs[i + 1][1], vx, vy, wz)
