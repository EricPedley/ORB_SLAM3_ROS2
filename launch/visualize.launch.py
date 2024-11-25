from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "objects_path",
                default_value="",
                description="Path to the objects file.",
            ),
            DeclareLaunchArgument(
                "cloud_path",
                default_value="",
                description="Path to the PCL cloud file.",
            ),
            Node(
                package="orb_slam3_ros2",
                executable="visualize_node",
                output="screen",
                # prefix="xterm -e gdb --args",
                parameters=[
                    {
                        "objects_path": LaunchConfiguration("objects_path"),
                        "cloud_path": LaunchConfiguration("cloud_path"),
                    }
                ],
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
                            "visualization.rviz",
                        ]
                    ),
                ],
            ),
        ],
    )
