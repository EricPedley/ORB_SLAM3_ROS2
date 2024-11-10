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
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rtabmap_db",
                default_value="",
                description="The path to the RTAB-Map database file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch RViz.",
            ),
            DeclareLaunchArgument(
                "export_images",
                default_value="false",
                description="Whether to export images from the database.",
            ),
            Node(
                package="orb_slam3_ros2",
                executable="rtabmap_database_extractor_node",
                output="screen",
                # prefix="xterm -e gdb --args",
                parameters=[
                    {
                        "rtabmap_db": LaunchConfiguration("rtabmap_db"),
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
                            "database.rviz",
                        ]
                    ),
                ],
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("use_rviz"), "' == 'true'"]
                    )
                ),
            ),
        ],
    )
