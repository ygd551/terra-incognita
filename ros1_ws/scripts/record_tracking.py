#!/usr/bin/env python3
"""Record how well the robot follows GBPlanner2's paths, read-only (ROS 1, sim time): every path sent to the path
follower (/pci_output_path/goal), the odometry (/bievr_lio/odom: BIEVR-LIO, or the ground truth with lio:=gt) and the
adapter's commands (/cmd_vel_stamped, drones) -> JSON for tools/tracking_report.py.
  docker exec gbplanner_ros1 bash -c "source /ws/devel/setup.bash && python3 /ws/scripts/record_tracking.py \
      /ws/logs/tracking.json [SIM_SECONDS=110]"        (ros1_ws/logs/tracking.json on the host)
"""
import json
import math
import sys

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from planner_msgs.msg import pathFollowerActionActionGoal

out, dur = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 110.0
rospy.init_node("record_tracking", anonymous=True)
d = {"paths": [], "odom": [], "cmd": []}


def xyz(p):
    p = p.pose.position if hasattr(p, "pose") else p.position
    return [p.x, p.y, p.z]


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    d["odom"].append([rospy.get_time(), p.x, p.y, p.z,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])


rospy.Subscriber("/pci_output_path/goal", pathFollowerActionActionGoal,
                 lambda m: d["paths"].append({"t": rospy.get_time(), "pts": [xyz(p) for p in m.goal.path]}))
rospy.Subscriber("/bievr_lio/odom", Odometry, on_odom)
rospy.Subscriber("/cmd_vel_stamped", TwistStamped, lambda m: d["cmd"].append(
    [rospy.get_time(), m.twist.linear.x, m.twist.linear.y, m.twist.linear.z, m.twist.angular.z]))
while rospy.get_time() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)
t0 = rospy.get_time()
print(f"recording {dur:.0f} s (sim time) ...", flush=True)
while not rospy.is_shutdown() and rospy.get_time() - t0 < dur:
    rospy.sleep(0.5)
json.dump(d, open(out, "w"))
print(f"{out}: {len(d['paths'])} paths, {len(d['odom'])} poses, {len(d['cmd'])} commands")
