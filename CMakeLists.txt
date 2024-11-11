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

set(USER_HOME $ENV{HOME})
# set(CMAKE_PREFIX_PATH "${USER_HOME}/.local/include/libpointmatcher/share/libpointmatcher/cmake;${CMAKE_PREFIX_PATH}")

message(WARNING "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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
find_package(realsense2_description REQUIRED)
find_package(xacro REQUIRED)
# find_package(Eigen3 3.3 REQUIRED) # Matched with Sophus
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL 1.12 REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)

find_package(ORB_SLAM3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(libpointmatcher_ros REQUIRED)
find_package(RTABMap REQUIRED)
# find_package(rtabmap_ros REQUIRED)
find_package(imu_filter_madgwick REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_map_server REQUIRED)

message("RTABMap_INCLUDE_DIRS: ${RTABMap_INCLUDE_DIRS}")
message("RTABMap_LIBRARIES: ${RTABMap_LIBRARIES}")

if(ORB_SLAM3_LIBRARY)
    message("Found ORB_SLAM3: ${ORB_SLAM3_LIBRARY}")
else()
    message("Could not find ORB_SLAM3 library")
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
  Pangolin
  image_transport
  pcl_ros
  pcl_conversions
  nav2_map_server
  yaml-cpp
  # rtabmap_ros
  libpointmatcher
  libpointmatcher_ros
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

add_executable(rtabmap_database_extractor_node
  src/rtabmap_database_extractor.cpp
)

add_executable(orb_camera_info_node
  src/orb_camera_info.cpp
)

ament_target_dependencies(imu_mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(rtabmap_database_extractor_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(orb_camera_info_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

set(ORB_SLAM3_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3)

include_directories(
    include
    ${ORB_SLAM3_ROOT_DIR}/include
    ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
    ${PCL_INCLUDE_DIRS}
    ${libpointmatcher_INCLUDE_DIRS}
    ${libpointmatcher_ros_INCLUDE_DIRS}
)

link_directories(
    include
)

target_include_directories(imu_mono_node_cpp PUBLIC
    ${ORB_SLAM3_INCLUDE_DIR}
)

target_link_libraries(imu_mono_node_cpp PUBLIC ${ORB_SLAM3_LIBRARY} ${PCL_LIBRARIES}
    ${libpointmatcher_LIBRARIES} ${libpointmatcher_ros_LIBRARIES} rtabmap::rtabmap)
target_link_libraries(rtabmap_database_extractor_node PUBLIC ${PCL_LIBRARIES} rtabmap::rtabmap)
target_link_libraries(orb_camera_info_node PUBLIC yaml-cpp)

install(TARGETS rtabmap_database_extractor_node imu_mono_node_cpp orb_camera_info_node
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

install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

install(
    DIRECTORY occupancy_grids/
    DESTINATION share/${PROJECT_NAME}/orb_maps
)

# install video files
install(
  DIRECTORY videos/
  DESTINATION share/${PROJECT_NAME}/videos
)

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/semantic_mapping.py
    DESTINATION lib/${PROJECT_NAME}
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
