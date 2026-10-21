"""Full autonomy stack for the Isaac Sim Go2 / ANYmal D (run inside the isaac_explore container, see ../explore.sh).

  ros2 launch /isaac_explore/launch/explore.launch.py [robot:=anymal_d] [planner:=false] [exploration:=false] [elevation:=false]
                                                      [rviz:=false] [lio:=gt] [speed:=1.0]

robot:= must match go2_sim.py --robot: lidar mount, footprint and heights come from config/robots.yaml.

lio:=gt replaces BIEVR-LIO by sim/gt_odom.py (ground truth, same topics/frames) to test planning without
localization errors.

Data flow:
  /lidar/points + /imu/data --BIEVR-LIO--> /bievr_lio/odom, /bievr_lio/points/registered, TF odom->imu
  --loam_interface--> /state_estimation, /registered_scan (frame map), TF map->sensor
  --terrain_analysis(+ext)--> /terrain_map(_ext) --TARE--> /way_point --local_planner--> /cmd_vel_stamped --> go2_sim
  /lidar/points + TF odom->imu->lidar --elevation_mapping_cupy--> /elevation_mapping_node/elevation_map_*
Robot-specific values: config/robots.yaml; tuned CMU launches: launch/cmu/*.launch; TARE: config/tare_go2.yaml.
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap

CFG = "/isaac_explore/config"


CMU = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cmu")  # copies of the CMU launches with Go2 values


def xml(path, **args):
    return IncludeLaunchDescription(FrontendLaunchDescriptionSource(path),
                                    launch_arguments={k: str(v) for k, v in args.items()}.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="go2", choices=["go2", "anymal_d"],
                              description="must match go2_sim.py --robot (config/robots.yaml)"),
        DeclareLaunchArgument("planner", default_value="true", description="TARE exploration + CMU local planner"),
        DeclareLaunchArgument("exploration", default_value="true",
                              description="TARE; false = everything else runs, publish your own /way_point"),
        DeclareLaunchArgument("elevation", default_value="true", description="elevation_mapping_cupy"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("speed", default_value="1.0",
                              description="top walking speed [m/s] (CMU maxSpeed = autonomySpeed); the policies are "
                                          "trained up to 1.0, and go2_sim clamps there"),
        DeclareLaunchArgument("lio", default_value="bievr", choices=["bievr", "gt"],
                              description="bievr = BIEVR-LIO, gt = ground truth from the sim (sim/gt_odom.py)"),
        SetParameter(name="use_sim_time", value=True),  # everything runs on go2_sim's /clock
        OpaqueFunction(function=stack),
    ])


def tare_override(robot, R):
    """Per-robot TARE values as a params file keyed by the node name: a plain dict would be applied as /** and lose
    against the node-specific entries of tare_go2.yaml."""
    path = f"/tmp/tare_{robot}.yaml"
    with open(path, "w") as f:
        yaml.safe_dump({"tare_planner_node": {"ros__parameters": {
            "kViewPointHeightFromTerrain": float(R["viewpoint_height"])}}}, f)
    return path


def stack(context):
    robot = LaunchConfiguration("robot").perform(context)
    R = yaml.safe_load(open(f"{CFG}/robots.yaml"))[robot]
    planner, elevation, rviz = LaunchConfiguration("planner"), LaunchConfiguration("elevation"), LaunchConfiguration("rviz")
    speed = float(LaunchConfiguration("speed").perform(context))
    return [
        Node(package="tf2_ros", executable="static_transform_publisher", name="imu_to_lidar",
             arguments=[*map(str, R["lidar_pos"]), "0", "0", "0", "imu", "lidar"]),
        Node(package="bievr_lio_ros2", executable="process_topics", name="bievr_lio", output="screen",
             condition=LaunchConfigurationEquals("lio", "bievr"),
             arguments=["--sensor_config_file", f"{CFG}/bievr_sim_{robot}.yaml", "--params_file", f"{CFG}/bievr_params.yaml"]),

        ExecuteProcess(cmd=["python3", "/isaac_explore/sim/gt_odom.py", "--robot", robot], name="gt_odom", output="screen",
                       condition=LaunchConfigurationEquals("lio", "gt")),

        GroupAction(condition=IfCondition(planner), actions=[
            Node(package="loam_interface", executable="loamInterface", name="loamInterface", output="screen",
                 parameters=[{"stateEstimationTopic": "/bievr_lio/odom",
                              "registeredScanTopic": "/bievr_lio/points/registered",
                              "flipStateEstimation": False, "flipRegisteredScan": False,
                              "sendTF": True, "reverseTF": False}]),
            xml(f"{CMU}/terrain_analysis.launch", vehicleHeight=R["vehicle_height"]),
            xml(f"{CMU}/terrain_analysis_ext.launch", checkTerrainConn="true", vehicleHeight=R["vehicle_height"]),
            xml(os.path.join(share("sensor_scan_generation"), "launch", "sensor_scan_generation.launch")),
            GroupAction([SetRemap(src="/cmd_vel", dst="/cmd_vel_stamped"),  # go2_sim takes TwistStamped here
                         # both equal: speed = min(maxSpeed, autonomySpeed), and the path library is scaled by
                         # autonomySpeed / maxSpeed (a lower ratio shortens the local paths)
                         xml(f"{CMU}/local_planner.launch", maxSpeed=speed, autonomySpeed=speed, autonomyMode="true",
                             twoWayDrive="false", vehicleLength=R["vehicle_length"], vehicleWidth=R["vehicle_width"],
                             obstacleHeightThre=R["obstacle_height"])]),  # forward only: fwd/back flip-flopping tipped the Go2 over
            # delayed: TARE exits if its first planning round sees time 0 (no /clock received yet), and the robot
            # must stand still while BIEVR estimates the IMU bias
            TimerAction(period=5.0, condition=IfCondition(LaunchConfiguration("exploration")), actions=[
                Node(package="tare_planner", executable="tare_planner_node", name="tare_planner_node", output="screen",
                     parameters=[f"{CFG}/tare_go2.yaml", tare_override(robot, R)])]),
            # exploration metrics: explored volume (0.5 m voxels of /registered_scan), travelled distance, time
            # -> /isaac_explore/logs/metrics_<date-time>.txt and trajectory_<date-time>.txt
            Node(package="visualization_tools", executable="visualizationTools", name="visualizationTools",
                 # it rewrites the first "/install" in these paths to "/src" (and crashes without one), so route
                 # through ros2_ws/install -> ros2_ws/src/../.. = /isaac_explore
                 parameters=[{"metricFile": "/isaac_explore/ros2_ws/install/../../logs/metrics",
                              "trajFile": "/isaac_explore/ros2_ws/install/../../logs/trajectory",
                              "mapFile": "/isaac_explore/logs/no_map.ply", "overallMapVoxelSize": 0.5, "exploredAreaVoxelSize": 0.3,
                              "exploredVolumeVoxelSize": 0.5, "transInterval": 0.2, "yawInterval": 10.0,
                              "overallMapDisplayInterval": 2, "exploredAreaDisplayInterval": 1}]),
            Node(package="rviz2", executable="rviz2", name="tare_rviz", condition=IfCondition(rviz),
                 arguments=["-d", os.path.join(share("tare_planner"), "tare_planner_ground.rviz")]),
        ]),

        Node(package="elevation_mapping_cupy", executable="elevation_mapping_node.py", name="elevation_mapping_node",
             output="screen", condition=IfCondition(elevation),
             parameters=[os.path.join(share("elevation_mapping_cupy"), "config", "core", "core_param.yaml"),
                         f"{CFG}/elevation_sim.yaml"]),
    ]
