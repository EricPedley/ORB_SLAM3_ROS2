from launch import LaunchDescription
from launch_ros.actions import Node
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
                "reference_map_file",
                default_value="changeme.pcd",
                description="The reference map file to localize against",
            ),
            DeclareLaunchArgument(
                "ip_address",
                default_value="172.17.0.3",
                description="The IP address of the luci docker container",
            ),
            DeclareLaunchArgument(
                "remote_launch",
                default_value="false",
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
            # Node(
            #     package="orb_slam3_ros2",
            #     executable="imu_mono_node_cpp",
            #     output="screen",
            #     # prefix="xterm -e gdb --args",
            #     parameters=[
            #         {
            #             "sensor_type": LaunchConfiguration("sensor_type"),
            #             "use_pangolin": LaunchConfiguration("use_pangolin"),
            #         }
            #     ],
            # ),
            Node(
                package="orb_slam3_ros2",
                executable="localize_node",
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                name="map_tf_broadcaster",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                name="orb_tf_broadcaster",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "map",
                    "orb_point_cloud",
                ],
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
        ],
    )
