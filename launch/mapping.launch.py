import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
)
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
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
            DeclareLaunchArgument(
                "ip_address",
                default_value="172.17.0.3",
                description="The IP address of the luci docker container",
            ),
            DeclareLaunchArgument(
                "remote_launch",
                default_value="false",
                description="Whether or not to launch the luci launch file on the docker container",
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
                "playback_bag",
                default_value="changeme",
                description="The rosbag to play during execution. If set, the \
                realsense2_camera node will not launch. Otherwise, nothing\
                will happen.",
            ),
            DeclareLaunchArgument(
                "record_bag",
                default_value="false",
                description="Whether or not to record a rosbag.",
            ),
            DeclareLaunchArgument(
                "bag_name",
                default_value=bag_name,
                description="The name of the bag if record_bag is true. By\
                        default, the name of the bag will be\
                        ORB_SLAM3_YYYY-MM-DD_HH-mm-ss",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("orb_slam3_ros2"),
                        "config",
                        "mapper_params_online_async.yaml",
                    ],
                ),
                description="Full path to the ROS2 parameters file to use for\
                        the slam_toolbox node",
            ),
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
                        [
                            "'",
                            LaunchConfiguration("playback_bag"),
                            "' == 'changeme'",
                        ]
                    )
                ),
            ),
            # Node(
            #     package="pointcloud_to_laserscan",
            #     executable="pointcloud_to_laserscan_node",
            #     remappings=[
            #         ("cloud_in", "tracked_point_cloud2"),
            #         # ("scan", "/orb_slam3_ros2/pointcloud"),
            #     ],
            #     parameters=[
            #         {
            #             "target_frame": "point_cloud",
            #             "transform_tolerance": 0.01,
            #             "min_height": 0.0,
            #             "max_height": 1.0,
            #             "angle_min": -1.5708,  # -M_PI/2
            #             "angle_max": 1.5708,  # M_PI/2
            #             "angle_increment": 0.0087,  # M_PI/360.0
            #             "scan_time": 1.0,
            #             "range_min": 0.45,
            #             "range_max": 100.0,
            #             "use_inf": True,
            #             "inf_epsilon": 1.0,
            #         }
            #     ],
            # ),
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
            # Node(
            #     package="octomap_server",
            #     executable="octomap_server_node",
            #     output="screen",
            #     parameters=[
            #         {
            #             "resolution": 0.05,
            #             "frame_id": "map",
            #             "sensor_model.max_range": 100.0,
            #             # "filter_ground_plane": True,
            #         }
            #     ],
            #     remappings=[("cloud_in", "orb_point_cloud2"),
            #                 ("projected_map", "map")],
            # ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("orb_slam3_ros2"),
                        "config",
                        "mapper_params_online_async.yaml",
                    ),
                    {
                        "use_sim_time": True,
                    },
                ],
                arguments=["--ros-args", "--log-level", "DEBUG"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(
                                [
                                    "xacro",
                                    " ",
                                    PathJoinSubstitution(
                                        [
                                            FindPackageShare(
                                                "realsense2_description"
                                            ),
                                            "urdf",
                                            "test_d435i_camera.urdf.xacro",
                                        ]
                                    ),
                                ]
                            ),
                            value_type=str,
                        ),
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
