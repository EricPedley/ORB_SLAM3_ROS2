cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ENV{PROJECT_PATH} "${PROJECT_SOURCE_DIR}")
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(${PCL_DEFINITIONS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap_server REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
# find_package(Eigen3 3.3 REQUIRED) # Matched with Sophus
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL 1.12 REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)

find_package(ORB_SLAM3 REQUIRED)

if(ORB_SLAM3_LIBRARY)
    message("Found ORB_SLAM3: ${ORB_SLAM3_LIBRARY}")
else()
    message("Could not find ORB_SLAM3 library")
endif()

if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found")
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  octomap_server
  nav_msgs
  apriltag
  apriltag_ros
  apriltag_msgs
  cv_bridge
  OpenCV
  Pangolin
  image_transport
  pcl_ros
  pcl_conversions
  ORB_SLAM3
)

# if(Eigen3_FOUND)
#     message(STATUS "Found Eigen3: ${Eigen3_INCLUDE_DIRS}")
#     message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
# else()
#     message(FATAL_ERROR "Could not find Eigen3")
# endif()

add_executable(imu_mono_node_cpp
  src/imu_mono_realsense.cpp
)

add_executable(capture_video
    src/capture_realsense_video.cpp
)

add_executable(visualize_point_cloud
    src/visualize_point_cloud.cpp
)

add_executable(apriltag_node
    src/apriltag_node.cpp
)

ament_target_dependencies(imu_mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(capture_video
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(visualize_point_cloud
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(apriltag_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

set(ORB_SLAM3_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3)

include_directories(
    include
    ${ORB_SLAM3_ROOT_DIR}/include
    ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    include
)

target_include_directories(imu_mono_node_cpp PUBLIC
    ${ORB_SLAM3_INCLUDE_DIR}
)

target_link_libraries(visualize_point_cloud PUBLIC ${PCL_LIBRARIES})
target_link_libraries(apriltag_node PUBLIC apriltag::apriltag)
target_link_libraries(imu_mono_node_cpp PUBLIC ${ORB_SLAM3_LIBRARY})
# target_link_libraries(mono_node_cpp PUBLIC ORB_SLAM3)
# target_link_libraries(example_mono_node_cpp PUBLIC ORB_SLAM3)

install(TARGETS apriltag_node visualize_point_cloud capture_video imu_mono_node_cpp # mono_node_cpp example_mono_node_cpp
    DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY ${ORB_SLAM3_INCLUDE_DIR}
        DESTINATION include
)

install(FILES ${ORB_SLAM3_LIBRARY}
        DESTINATION lib
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
        DESTINATION lib
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
        DESTINATION lib
)

# install bag files
install(
  DIRECTORY bags/
  DESTINATION share/${PROJECT_NAME}/bags
)

# install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# install video files
install(
  DIRECTORY videos/
  DESTINATION share/${PROJECT_NAME}/videos
)

# # install EuRoC dataset
# install(
#   DIRECTORY TEST_DATASET/
#   DESTINATION share/${PROJECT_NAME}/TEST_DATASET)

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
