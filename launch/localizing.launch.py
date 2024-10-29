from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "reference_map_file",
                default_value="changeme.pcd",
                description="The path to the reference map file",
            ),
            DeclareLaunchArgument(
                "playback_bag",
                default_value="changeme",
                description="The rosbag to play during execution. If set, the \
                realsense2_camera node will not launch. Otherwise, nothing\
                will happen.",
            ),
            DeclareLaunchArgument(
                "sensor_type",
                default_value="imu-monocular",
                description="The mode which ORB_SLAM3 will run in.",
            ),
            DeclareLaunchArgument(
                "use_pangolin",
                default_value="true",
                description="Whether to use Pangolin for visualization.",
            ),
            DeclareLaunchArgument(
                "launch_orbslam3",
                default_value="false",
                description="Whether to launch ORB_SLAM3.",
            ),
            Node(
                package="orb_slam3_ros2",
                executable="icp_localization_node",
                output="screen",
                # prefix="xterm -e gdb --args",
                parameters=[
                    {
                        "reference_map_file": LaunchConfiguration(
                            "reference_map_file"
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orb_slam3_ros2"),
                            "launch",
                            "mapping.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "playback_bag": LaunchConfiguration("playback_bag"),
                    "use_rviz": "false",
                }.items(),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("launch_orbslam3"),
                            "' == 'true'",
                        ]
                    )
                ),
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
                            "localize.rviz",
                        ]
                    ),
                ],
            ),
        ],
    )
