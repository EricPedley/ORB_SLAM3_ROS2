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
