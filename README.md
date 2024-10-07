# ORB_SLAM3_ROS2
Implementing ORB_SLAM3 in ROS2 humble

### Getting started

To get started, we have to build orbslam3 and its dependencies.

cd into the ORB_SLAM3 directory and run the ```build.sh``` script. Make sure the
script has execute permissions.

```sh
cd ORB_SLAM3
./build.sh
```

Note: You may need to go into the cmake files for Sophus and g2o and change the
directory for eigen. I will soon write a guide on installing different versions
of eigen into a prefix directory for use with orbslam3.

Now you can source ros humble, and the build the project. cd into your ros2
workspace and type
```sh
colcon build
```

### Running the project

Run this command in the directory where you just ran colcon build:
```sh
source install/setup.bash
```
Depending on your shell you will need to source a different file.

The command we use to run SLAM using a D435i RealSense camera is:
```sh
ros2 launch orb_slam3_ros2 mapping.launch.xml
```

There are a few command line arguments for this launch file, here are the
most important ones:
* **use_live_feed (default: true)**: Specifies whether to try and use the image stream from the
RealSense camera. If set to false, you must specify a file name in the
video_name argument. See the following bullet
* **sensor_type (default: imu-monocular)**: Specifies the orbslam3 mode you want to use. Currently, the
only two options for this argument, imu-monocular and monocular.
* **use_pangolin (default: true)**: Specifies whether to use Pangolin to display
the orbslam3 map.
* **video_name (default: "ChangeMe.mp4")**: Specifies the video to use to run
orbslam3 on. This can only be use if the use_live_feed argument is set to false.
* **bag_name (default: "changeme")**: Specifies the ros2 bag file to use instead
of a live camera feed from the RealSense camera.
