# ORB_SLAM3_ROS2
Implementing ORB_SLAM3 in ROS 2 humble with some bonus features.

This repository is meant for running ORB_SLAM3 in ROS 2 humble with a D435i Realsense
camera. Eventually, support for any camera+imu setup will be added. If you're looking
to run ORB_SLAM3 on a dataset using ROS 2, I suggest you look at other repositories.

This project is only set up for monocular and imu-monocular modes in orb_slam3
at the moment. I can add stereo and RGBD modes if there is interest.

### Building the project

#### Setting up your workspace
The first thing I will do is explain how I set up my ROS 2 workspace for this project.

Make a directory for your ROS 2 workspace, then cd into it:
```sh
mkdir ws && cd ws
```
inside the ```ws``` directory, clone this repository and its submodules:
```sh
git clone --recurse-submodules -b orb_slam3_ros2 https://github.com/gjcliff/ORB_SLAM3_ROS2.git
```
#### Building ORB_SLAM3
Next, we have to build orbslam3 and its dependencies.

cd into the ORB_SLAM3 directory and run the ```build.sh``` script. Make sure the
script has execute permissions.
```sh
cd ORB_SLAM3_ROS2/ORB_SLAM3/
./build.sh
```
Note: By default, ORB_SLAM3 calls for Eigen 3.1 for g2o, but Eigen 3.3 for itself.
In order to accomplish this I installed Eigen 3.1 and 3.3 into a CMake prefix
directory and manually include them. This is especially necessary for Eigen 3.1
because it doesn't come with any CMake modules.

TODO: Make a dockerfile for the project so that dependencies are set up by default.
TODO: Make a guide for installing and using different versions of Eigen from CMake
prefix directories.

#### Building the ROS 2 project
Now you can source ros humble, and the build the project. cd into your ROS 2
workspace and type
```sh
cd -
colcon build
source install/setup.bash
```
### Running the project

There are multiple ways to use this project.

#### Basic Usage

The command we use to run SLAM using a D435i RealSense camera is:
```sh
ros2 launch orb_slam3_ros2 mapping.launch.xml
```
This will launch the ORB_SLAM3 system in imu-monocular mode by default. You should
be able to see Pangolin launch and start displaying the point cloud, and you should
also see an opencv window pop up that displays the keypoints being captured by
ORB_SLAM3. You can walk around with your RealSense and begin to build a map of
your environment. If you want to save your map, you have a few options. This
first is to make the following ROS 2 service call:
```sh
ros2 service call /slam_service std_srvs/srv/Empty {}\
```
This will save a map in the ```ORB_SLAM3_ROS2/maps``` directory in a file named
with the current date and time. This file is a .pcd file, which is from the [Point Cloud Library](https://pointclouds.org/).

We can use this file to retroactively view the map we just created in RVIZ.
To do this, we can run the following command:
```sh
ros2 launch orb_slam3_ros2 visualize_point_cloud.launch.xml pcl_file:=<name_of_pcl_file>
```
Make sure you use only the name of the file, not the full file path.

#### Other Features

There are a couple other features that can be used through arguments to the
```mapping.launch.py``` file. Here is a list:
```sh
'sensor_type':
    The mode which ORB_SLAM3 will run in.
    (default: 'imu-monocular')

'use_pangolin':
    Whether to use Pangolin for visualization.
    (default: 'true')

'playback_bag':
    The rosbag to play during execution. If set, the realsense2_camera node will not launch. Otherwise, nothing will happen.
    (default: 'changeme')

'record_bag':
    Whether or not to record a rosbag.
    (default: 'false')

'bag_name':
    The name of the bag if record_bag is true. By default, the name of the bag will be ORB_SLAM3_YYYY-MM-DD_HH-mm-ss
    (default: 'ORB_SLAM3_2024-10-15_00-42-34')
```
My general workflow is to record a bag, and then play it back over and over again
while I tweak things.

Another feature is the ability to build 2D occupancy grids from the 3D point clouds
created by ORB_SLAM3. By default, the point cloud is fed into an octomap_server
node, which creates an octomap from the data in addition to a 2D occupancy grid.
My plan is to soon integrate slam_toolbox with this project so that these 2D occupancy
grids can be used to localize with.

### Troubleshooting
1. ORB_SLAM3 keeps resetting the map on its own.
    * Sometimes the map keeps getting lost over and over again over the course of a singular
    run. The best option is just to kill the program and try again.
2. IMU Initialization keeps failing
    * Try and make sure you're moving at the start of the program so that the IMU
    has enough data to initialize. Look for VIBA 1 and VIBA 2 in the terminal output
    (Visual Inertial Bundle Adjustment). However I've noticed that once VIBA 1
    has been completed, you'll likely be able to keep the map even if you stand
    relatively still.
3. Maps look strange or too unlike the real environment
    * I suggest calibrating your camera and IMU. I've provided files, scripts,
    and instructions for doing this [here](./camera_calibration/README.md)
