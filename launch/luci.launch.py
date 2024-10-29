import os

from launch import LaunchDescription, LaunchContext
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    # IncludeLaunchDescription,
    ExecuteProcess,
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ip_addr",
                default_value="172.17.0.3",
                description="The IP address of the luci docker container",
            ),
            DeclareLaunchArgument(
                "ssh_cmd",
                default_value=["root@", LaunchConfiguration("ip_addr")],
                description="The ssh command of the luci docker container",
            ),
            DeclareLaunchArgument(
                "remote_launch",
                default_value="true",
                description="Whether or not to launch the luci launch file on\
                        the docker container",
            ),
            DeclareLaunchArgument(
                "playback_bag",
                default_value="changeme",
                description="The rosbag to play during execution. If set, the \
                realsense2_camera node will not launch. Otherwise, nothing\
                will happen.",
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
                        [
                            "'",
                            LaunchConfiguration("playback_bag"),
                            "' != 'changeme'",
                        ]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "ssh",
                    "-t",
                    LaunchConfiguration("ssh_cmd"),
                    "bash -i -c 'ros2 launch awl_navigation luci_nav.launch.py\
                            slam_toolbox:=true'",
                ],
                shell=True,
            ),
        ],
    )
