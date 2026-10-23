#!/usr/bin/env python3
"""Drive the simulated robot straight up one lane of make_ramps.py's scene and report how it did.
  (isaac_explore container, sim running on that lane)  python3 ramp_test.py ANGLE [SPEED=0.8] [SIM_SECONDS=50]
Walks along +x with /cmd_vel; reads /ground_truth/odom. Lane geometry as in make_ramps.py: up-ramp from x = 0 to d,
plateau to d + 3, down-ramp to 2d + 3 (d = 1.5 m / tan(angle))."""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

RISE, TOP = 1.5, 3.0


def main():
    angle = float(sys.argv[1])
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.8
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 50.0
    d = RISE / math.tan(math.radians(angle))
    rclpy.init()
    node = rclpy.create_node("ramp_test")
    pub = node.create_publisher(Twist, "/cmd_vel", 5)
    s = {"t0": None, "t": 0.0, "x": None, "max_x": -1e9, "max_z": -1e9, "max_tilt": 0.0, "falls": 0, "down": False,
         "t_top": None, "t_end": None}

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        s["t0"] = t if s["t0"] is None else s["t0"]
        s["t"], s["x"] = t - s["t0"], p.x
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
        s["max_x"], s["max_z"], s["max_tilt"] = max(s["max_x"], p.x), max(s["max_z"], p.z), max(s["max_tilt"], tilt)
        if (tilt > 60) != s["down"]:
            s["down"] = tilt > 60
            s["falls"] += s["down"]
        if s["t_top"] is None and p.x > d + 0.5 and not s["down"]:
            s["t_top"] = s["t"]
        if s["t_end"] is None and p.x > 2 * d + TOP + 1.0 and not s["down"]:
            s["t_end"] = s["t"]

    node.create_subscription(Odometry, "/ground_truth/odom", on_odom, 10)
    cmd = Twist()
    cmd.linear.x = speed
    while rclpy.ok() and (s["t0"] is None or s["t"] < duration) and s["t_end"] is None:
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(Twist())
    top = "%.0f s" % s["t_top"] if s["t_top"] is not None else "no"
    end = "%.0f s" % s["t_end"] if s["t_end"] is not None else "no"
    print("%4.0f deg: up-ramp to x %.1f, top reached %s, down and off at x %.1f %s | furthest x %.1f, max base z %.2f, "
          "max tilt %.0f deg, falls %d" % (angle, d, top, 2 * d + TOP, end, s["max_x"], s["max_z"], s["max_tilt"],
                                            s["falls"]), flush=True)


if __name__ == "__main__":
    main()
