#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/point_field__struct.hpp>
#include <string>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;
// using std::placeholders::_1;

class VisualizePointCloud : public rclcpp::Node {
public:
  VisualizePointCloud() : Node("visualize_point_cloud")
  {
    // declare parameters
    declare_parameter("map_file_name",
                      "2024-05-05_01:44:37 PM_graham_apt_imu_map.csv");
    declare_parameter("pcl_file", "changeme.pcd");
    declare_parameter("z_threshold", 0.1);
    std::string file_name = get_parameter("map_file_name").as_string();
    pcl_file = get_parameter("pcl_file").as_string();
    z_threshold = get_parameter("z_threshold").as_double();

    map_file_path =
      static_cast<std::string>(PROJECT_PATH) + "/maps/" + file_name;

    // create publishers
    // point_cloud_publisher =
    // create_publisher<sensor_msgs::msg::PointCloud>("orb_point_cloud", 10);

    point_cloud2_publisher =
      create_publisher<sensor_msgs::msg::PointCloud2>("orb_point_cloud2", 10);

    // Initialize the transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer = create_wall_timer(
      500ms, std::bind(&VisualizePointCloud::timer_callback, this));

    load_point_cloud();
  }

private:
  void load_point_cloud()
  {
    RCLCPP_INFO_STREAM(get_logger(),
                       "loading point cloud from "
                         << std::string(PROJECT_PATH) + "/maps/" + pcl_file);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::io::loadPCDFile(std::string(PROJECT_PATH) + "/maps/" + pcl_file,
                         pcl_cloud);

    RCLCPP_INFO_STREAM(get_logger(),
                       "loaded " << pcl_cloud.points.size() << " points");

    // now i can filter the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

    // statistical outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.5);
    sor.filter(*cloud_filtered);

    // voxel grid filter
    // pcl::VoxelGrid<pcl::PointXYZ> vg;
    // vg.setInputCloud(cloud_ptr);
    // vg.setLeafSize(0.05f, 0.05f, 0.05f);
    // vg.filter(*cloud_filtered);

    // convex hull
    // pcl::ConvexHull<pcl::PointXYZ> chull;
    // chull.setInputCloud(cloud_filtered);
    // chull.reconstruct(*cloud_filtered);

    // moving least square
    // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    // mls.setInputCloud(cloud_ptr);
    // mls.setSearchRadius(0.03);
    // mls.setPolynomialOrder(2);
    // mls.setUpsamplingMethod(
    //   pcl::MovingLeastSquares<pcl::PointXYZ,
    //                           pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.025);
    // mls.setUpsamplingStepSize(0.015);
    // mls.process(*cloud_filtered);

    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setInputCloud(cloud_filtered);

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(
      new pcl::ConditionAnd<pcl::PointXYZ>());

    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, z_threshold)));

    condrem.setCondition(condition);
    condrem.setKeepOrganized(true);
    condrem.filter(*cloud_filtered);

    pcl::toROSMsg(*cloud_filtered, point_cloud2);
    point_cloud2.header.frame_id = "point_cloud";
  }
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "point_cloud";

    t.transform.translation.z = 2;
    tf_broadcaster->sendTransform(t);

    // point_cloud.header.stamp = get_clock()->now();
    point_cloud2.header.stamp = get_clock()->now();
    // point_cloud_publisher->publish(point_cloud);
    point_cloud2_publisher->publish(point_cloud2);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr
    point_cloud_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud2_publisher;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    occupancy_grid_sub;

  sensor_msgs::msg::PointCloud point_cloud;
  sensor_msgs::msg::PointCloud2 point_cloud2;

  tf2::Quaternion initial_orientation;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  std::string map_file_path;
  std::string pcl_file;
  double z_threshold;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizePointCloud>());
  rclcpp::shutdown();
  return 0;
}
