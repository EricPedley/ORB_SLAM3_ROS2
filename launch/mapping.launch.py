import os
from datetime import datetime

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)


def generate_launch_description():
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_name = f"ORB_SLAM3_{current_time}"
    return LaunchDescription(
        [
            DeclareLaunchArgument("sensor_type", default_value="imu-monocular"),
            DeclareLaunchArgument("use_pangolin", default_value="true"),
            DeclareLaunchArgument("playback_bag", default_value="changeme"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_name", default_value=bag_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("realsense2_camera"),
                            "launch",
                            "rs_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "initial_reset": "true",
                    "rgb_camera.color_profile": "640x480x30",
                    "enable_accel": "true",
                    "enable_gyro": "true",
                    "unite_imu_method": "2",
                    "enable_depth": "false",
                }.items(),
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("playback_bag"), "' == 'changeme'"]
                    )
                ),
            ),
            Node(
                package="orb_slam3_ros2",
                executable="imu_mono_node_cpp",
                output="screen",
                # prefix="xterm -e gdb --args",
                parameters=[
                    {
                        "sensor_type": LaunchConfiguration("sensor_type"),
                        "use_pangolin": LaunchConfiguration("use_pangolin"),
                    }
                ],
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                output="screen",
                parameters=[
                    {
                        "resolution": 0.05,
                        "frame_id": "map",
                        "sensor_model.max_range": 5.0,
                        # "filter_ground_plane": True,
                    }
                ],
                remappings=[("cloud_in", "orb_point_cloud2")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orb_slam3_ros2"),
                            "config",
                            "point_cloud.rviz",
                        ]
                    ),
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orb_slam3_ros2"),
                            "bags",
                            LaunchConfiguration("playback_bag"),
                        ]
                    ),
                ],
                shell=True,
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("playback_bag"), "' != 'changeme'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    PathJoinSubstitution(
                        [
                            "./ORB_SLAM3_ROS2",
                            "bags",
                            LaunchConfiguration("bag_name"),
                        ]
                    ),
                    "/camera/camera/imu",
                    "/camera/camera/color/image_raw",
                ],
                shell=True,
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("record_bag"), "' == 'true'"]
                    )
                ),
            ),
        ],
    )
