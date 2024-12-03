# ORB_SLAM3_ROS2
Implementing ORB_SLAM3 in ROS 2 humble with some bonus features.

This repository is meant for running ORB_SLAM3 in ROS 2 humble with a D435i Realsense
camera. Eventually, support for any camera+imu setup will be added. If you're looking
to run ORB_SLAM3 on a dataset using ROS 2, I suggest you look at other repositories.

This project is only set up for monocular and imu-monocular modes in orb_slam3
at the moment. I can add stereo and RGBD modes if there is interest.

### Table of Contents
1. [Building the project](#building-the-project)
2. [Running the project](#running-the-project)
3. [Troubleshooting](#troubleshooting)

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

#### Building Docker
TODO

#### ORB_SLAM3

The command to run VIO SLAM using a D435i RealSense camera is:
```sh
ros2 launch orb_slam3_ros2 mapping.launch.xml
```
This will launch the ORB_SLAM3 system in imu-monocular mode by default. You should
see rviz pop up, and then shortly a Pangolin and an opencv window. You should
also see the 3D point cloud from ORB_SLAM3 and an occupancy grid being built
in RVIZ. The map you create will automatically be saved as a filtered
point cloud for the point cloud library (PCL). These files are stored in the
```maps``` directory.

You can record and play back rosbags with arguments to the launch file:
```sh
'record_bag':
    Whether or not to record a rosbag.
    (default: 'false')

'bag_name':
    The name of the bag if record_bag is true. By default, the name of the bag
    will be ORB_SLAM3_YYYY-MM-DD_hh-mm-ss
    (default: 'ORB_SLAM3_YYYY-MM-DD_hh-mm-ss')

'playback_bag':
    The rosbag to play during execution. If set, the realsense2_camera node
    will not launch. Otherwise, nothing will happen.
    (default: 'changeme')
```
Bags are recorded to the ```bags``` directory. The playbag_back argument shouldn't
be a full path to the bag, just the name of it.

### Visualizing The Map
You can visualize outputs from ORB_SLAM3 or from [RTABMap_Semantic_Mapping](https://github.com/gjcliff/RTABMap_Semantic_Mapping) with
the command:
```sh
ros2 launch orb_slam3_ros2 visualize.launch.py output_file_name:=<output_file_name>
```
These files should be placed in the ```output/``` directory if you are copying
them from RTABMap_Semantic_Mapping, and if you are running the ORB_SLAM3 node
then they'll automatically be placed in the ```output/``` directory for you.

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
