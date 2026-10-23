#!/usr/bin/env python3
"""Glue between GBPlanner2 (ROS 1) and the isaac_explore stack (ROS 2, reached through ros1_bridge).

Bridged in from ROS 2:  /bievr_lio/odom (frame odom, child imu), /bievr_lio/points/registered (frame odom, deskewed,
                        stamped like the odometry), /clock
Bridged out to ROS 2:   /way_point (geometry_msgs/PointStamped, frame map, which equals odom) for the CMU local planner

On the ROS 1 side this node
  - publishes TF odom -> imu (from the odometry) and imu -> lidar (lidar mount from config/robots.yaml)
  - republishes the registered scan in the lidar frame at its own pose as /gbplanner/input_pointcloud: voxblox then
    ray-casts from the sensor origin with a TF that matches the stamp exactly
  - is the robot's path follower, i.e. the action server pci_output_path (planner_msgs/pathFollowerAction): it feeds the
    CMU local planner a look-ahead point along GBPlanner's path and reports success at the end of the path. When the
    robot makes no progress it reports success too (result.success = false): pci_general halts the whole exploration
    on any other outcome, while success makes GBPlanner re-plan from where the robot is. Nothing else is fed back:
    the baseline stays GBPlanner as published (README, GBPlanner2)
  - logs every path it gets (start, end, length, robot position)
  - starts exploring: first pci's init motion (service pci_initialization_trigger: walk init_motion/x_forward m along
    +x), then auto mode (service planner_control_interface_trigger). The lidar (+-22.5 deg, ~0.8 m up) never sees the
    ground within ~2 m of the robot, and GBPlanner's ground-robot graph only grows over observed ground: from the
    spawn pose every edge would be "hanging" (No feasible path was found)

  rosrun-free: python3 isaac_adapter.py [--robot anymal_d] [--selftest]   (needs /use_sim_time and the built workspace)
"""
import argparse
import threading

import numpy as np

LOOKAHEAD = 5.0      # m along the path, like TARE's kLookAheadDistance
GOAL_TOL = 1.0       # m to the end of the path = done
STALL_TIME = 20.0    # s (sim) without STALL_PROGRESS m of progress along the path -> give the path up
STALL_PROGRESS = 0.5


def quat_to_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def world_to_lidar(points, t, rot, lidar_pos):
    """points (N,3) in odom -> lidar frame, given the imu pose (t, rot) in odom and the lidar mount (no rotation)."""
    return (points - t) @ rot - lidar_pos


def lookahead(path, pos, start, dist):
    """Index of the path vertex closest to pos (searching from start on) and the point dist further along the path."""
    i = start + int(np.argmin(np.linalg.norm(path[start:, :2] - pos[:2], axis=1)))
    seg = np.linalg.norm(np.diff(path[i:], axis=0), axis=1)
    acc = np.cumsum(seg)
    k = int(np.searchsorted(acc, dist))
    if k >= len(seg):
        return i, path[-1], float(acc[-1]) if len(acc) else 0.0
    before = acc[k] - seg[k]
    frac = (dist - before) / seg[k] if seg[k] > 0 else 0.0
    return i, path[i + k] + frac * (path[i + k + 1] - path[i + k]), float(acc[-1])


def selftest():
    path = np.array([[0.0, 0, 0], [4, 0, 0], [4, 4, 0], [4, 8, 0]])
    i, p, rem = lookahead(path, np.array([0.1, 0.2, 0]), 0, 5.0)
    assert i == 0 and np.allclose(p, [4, 1, 0]) and rem == 12.0, (i, p, rem)
    i, p, _ = lookahead(path, np.array([4.0, 7.0, 0]), 1, 5.0)
    assert i == 3 and np.allclose(p, [4, 8, 0]), (i, p)
    rot = quat_to_rot(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4))  # yaw 90 deg
    q = world_to_lidar(np.array([[1.0, 3.0, 1.25]]), np.array([1.0, 2.0, 1.0]), rot, np.array([0, 0, 0.25]))
    assert np.allclose(q, [[1.0, 0.0, 0.0]]), q   # 1 m ahead of a robot facing +y
    print("selftest ok")


def main(robot):
    import actionlib
    import rospy
    import tf2_ros
    import yaml
    from geometry_msgs.msg import PointStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from planner_msgs.msg import pathFollowerActionAction, pathFollowerActionFeedback, pathFollowerActionResult
    from planner_msgs.srv import pci_initialization, pci_trigger, pci_triggerRequest
    from sensor_msgs.msg import PointCloud2, PointField

    rospy.init_node("isaac_adapter")
    lidar_pos = np.array(yaml.safe_load(open("/isaac_explore_config/robots.yaml"))[robot]["lidar_pos"], float)
    tf_pub, static_pub = tf2_ros.TransformBroadcaster(), tf2_ros.StaticTransformBroadcaster()
    st = TransformStamped()
    st.header.frame_id, st.child_frame_id = "imu", "lidar"
    st.transform.translation.x, st.transform.translation.y, st.transform.translation.z = lidar_pos
    st.transform.rotation.w = 1.0
    static_pub.sendTransform(st)
    cloud_pub = rospy.Publisher("/gbplanner/input_pointcloud", PointCloud2, queue_size=2)
    wp_pub = rospy.Publisher("/way_point", PointStamped, queue_size=2)
    lock = threading.Lock()
    poses = {}      # odom stamp (ns) -> (t, rot): the registered scan carries the same stamp
    pending = []    # scans that arrived before their odometry
    state = {"pos": None, "paths_done": 0}
    fields = [PointField(n, 4 * i, PointField.FLOAT32, 1) for i, n in enumerate("xyz")]

    def publish_scan(msg, t, rot):
        off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
        a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        pts = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
        pts = world_to_lidar(pts[np.isfinite(pts).all(1)], t, rot, lidar_pos).astype(np.float32)
        out = PointCloud2(height=1, width=len(pts), fields=fields, is_bigendian=False, point_step=12,
                          row_step=12 * len(pts), is_dense=True, data=pts.tobytes())
        out.header.stamp, out.header.frame_id = msg.header.stamp, "lidar"
        cloud_pub.publish(out)

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t, rot = np.array([p.x, p.y, p.z]), quat_to_rot(q.x, q.y, q.z, q.w)
        tr = TransformStamped()
        tr.header.stamp, tr.header.frame_id, tr.child_frame_id = m.header.stamp, "odom", "imu"
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = p.x, p.y, p.z
        tr.transform.rotation = q
        tf_pub.sendTransform(tr)
        with lock:
            state["pos"] = t
            poses[m.header.stamp.to_nsec()] = (t, rot)
            while len(poses) > 50:
                poses.pop(next(iter(poses)))
            ready = [s for s in pending if s.header.stamp.to_nsec() in poses]
            for s in ready:
                pending.remove(s)
        for s in ready:
            publish_scan(s, *poses[s.header.stamp.to_nsec()])

    def on_scan(m):
        with lock:
            pose = poses.get(m.header.stamp.to_nsec())
            if pose is None:
                pending.append(m)
                del pending[:-3]
                return
        publish_scan(m, *pose)

    rospy.Subscriber("/bievr_lio/odom", Odometry, on_odom, queue_size=20)
    rospy.Subscriber("/bievr_lio/points/registered", PointCloud2, on_scan, queue_size=3, buff_size=2 ** 24)

    def execute(goal):
        try:
            follow(goal)
        finally:
            state["paths_done"] += 1

    def follow(goal):
        path = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in goal.path])
        done = pathFollowerActionResult()
        if len(path) < 2 or state["pos"] is None:
            done.success = len(path) > 0
            server.set_succeeded(done)
            return
        rospy.loginfo("isaac_adapter: path %d poses %.1f m (%.1f, %.1f) -> (%.1f, %.1f), robot at (%.1f, %.1f)",
                      len(path), np.linalg.norm(np.diff(path, axis=0), axis=1).sum(), *path[0, :2], *path[-1, :2],
                      *state["pos"][:2])
        idx, best, best_t = 0, float("inf"), rospy.get_time()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if server.is_preempt_requested():
                server.set_preempted()
                return
            pos = state["pos"]
            idx, target, remaining = lookahead(path, pos, idx, LOOKAHEAD)
            remaining += float(np.linalg.norm(path[idx, :2] - pos[:2]))
            wp = PointStamped()
            wp.header.stamp, wp.header.frame_id = rospy.Time.now(), "map"
            wp.point.x, wp.point.y, wp.point.z = target
            wp_pub.publish(wp)
            fb = pathFollowerActionFeedback()
            fb.remaining_waypoints, fb.dist_to_goal = len(path) - idx, remaining
            fb.estimated_time_remaining = remaining / 1.0
            server.publish_feedback(fb)
            if np.linalg.norm(path[-1, :2] - pos[:2]) < GOAL_TOL:
                done.success = True
                server.set_succeeded(done)
                return
            if remaining < best - STALL_PROGRESS:
                best, best_t = remaining, rospy.get_time()
            elif rospy.get_time() - best_t > STALL_TIME:
                rospy.logwarn("isaac_adapter: no progress for %.0f s (%.1f m left): giving the path up, re-plan",
                              STALL_TIME, remaining)
                done.success = False
                server.set_succeeded(done)   # not aborted: pci_general stops exploring after an abort
                return
            rate.sleep()

    server = actionlib.SimpleActionServer("pci_output_path", pathFollowerActionAction, execute_cb=execute,
                                          auto_start=False)
    server.start()

    def start_exploration():
        while not rospy.is_shutdown() and state["pos"] is None:
            rospy.sleep(1.0)
        rospy.wait_for_service("pci_initialization_trigger")
        rospy.ServiceProxy("pci_initialization_trigger", pci_initialization)()
        rospy.loginfo("isaac_adapter: init motion (walk forward to see the ground around the robot)")
        while not rospy.is_shutdown() and state["paths_done"] == 0:
            rospy.sleep(0.5)
        rospy.wait_for_service("planner_control_interface_trigger")
        trigger = rospy.ServiceProxy("planner_control_interface_trigger", pci_trigger)
        while not rospy.is_shutdown():
            if trigger(pci_triggerRequest(not_exe_path=False, set_auto=True, bound_mode=0, vel_max=1.0)).success:
                rospy.loginfo("isaac_adapter: exploration started (pci in auto mode)")
                return
            rospy.sleep(2.0)

    threading.Thread(target=start_exploration, daemon=True).start()
    rospy.loginfo("isaac_adapter: up (robot %s, lidar at %s)", robot, lidar_pos.tolist())
    rospy.spin()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="anymal_d")
    ap.add_argument("--selftest", action="store_true")
    args, _ = ap.parse_known_args()
    selftest() if args.selftest else main(args.robot)
