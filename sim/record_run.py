"""Record an exploration run: LIO pose (/state_estimation_at_scan, what TARE uses), ground truth, way points and
commands, all stamped in sim time, to a CSV; report pose jumps as they happen.
python3 record_run.py OUT.csv [seconds]"""
import csv, math, sys, rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
out, dur = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
rclpy.init(); n = rclpy.create_node("record_run")
f = open(out, "w", newline=""); w = csv.writer(f); w.writerow(["t", "topic", "x", "y", "z", "a", "b"])
st = {"t0": None, "t": 0.0, "last": {}}
def t_of(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def odom(topic):
    def cb(m):
        t, p = t_of(m.header), m.pose.pose.position
        st["t0"] = st["t0"] if st["t0"] is not None else t
        st["t"] = max(st["t"], t)
        w.writerow([f"{t:.3f}", topic, f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}", "", ""])
        q = st["last"].get(topic)
        if q and math.dist(q[1:], (p.x, p.y, p.z)) > 1.0 and t - q[0] < 1.0:
            print(f"[jump] {topic} t={t - st['t0']:.1f}s moved {math.dist(q[1:], (p.x, p.y, p.z)):.2f} m in {t - q[0]:.2f} s "
                  f"({q[1]:.2f},{q[2]:.2f},{q[3]:.2f}) -> ({p.x:.2f},{p.y:.2f},{p.z:.2f})", flush=True)
        st["last"][topic] = (t, p.x, p.y, p.z)
    return cb
for tp in ("/state_estimation_at_scan", "/ground_truth/odom"):
    n.create_subscription(Odometry, tp, odom(tp), 50)
n.create_subscription(PointStamped, "/way_point",
                      lambda m: w.writerow([f"{t_of(m.header):.3f}", "/way_point", f"{m.point.x:.3f}", f"{m.point.y:.3f}", f"{m.point.z:.3f}", "", ""]), 10)
n.create_subscription(TwistStamped, "/cmd_vel_stamped",
                      lambda m: w.writerow([f"{t_of(m.header):.3f}", "/cmd_vel_stamped", "", "", "", f"{m.twist.linear.x:.2f}", f"{m.twist.angular.z:.2f}"]), 10)
try:
    while st["t0"] is None or st["t"] - st["t0"] < dur:
        rclpy.spin_once(n, timeout_sec=1.0)
except KeyboardInterrupt:  # run_trial.sh stops us with SIGINT once TARE reports completion
    pass
finally:
    f.close()
    print(f"[record] {out}: {st['t'] - (st['t0'] or st['t']):.0f} s recorded", flush=True)
