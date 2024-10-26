#include <functional>
#include <memory>
#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/PointMatcher_ROS.h"
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ICP : public rclcpp::Node {
public:
  ICP() : Node("icp_node")
  {
    // create publishers
    transformed_point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("transformed_point_cloud",
                                                      10);
    occupancy_grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "transformed_occupancy_grid", 10);

    // create subscriptions
    reference_point_cloud_subscriber_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        "reference_point_cloud", 10,
        std::bind(&ICP::reference_point_cloud_callback, this, _1));
    data_point_cloud_subscriber_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        "data_point_cloud", 10,
        std::bind(&ICP::data_point_cloud_callback, this, _1));

    // create tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create timer
    timer_ = create_wall_timer(500ms, std::bind(&ICP::timer_callback, this));

    // initialize variables
    data_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    reference_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

private:
  void reference_point_cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::fromROSMsg(*msg, reference_pcl_cloud_);
    reference_occupancy_grid_ = point_cloud_to_occupancy_grid(
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(reference_pcl_cloud_));
  }
  void
  data_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::fromROSMsg(*msg, data_pcl_cloud_);
    data_occupancy_grid_ = point_cloud_to_occupancy_grid(
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(data_pcl_cloud_));
  }
  void match_point_cloud()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr data_og_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_og_cloud_ptr;
    data_og_cloud_ptr = occupancy_grid_to_2d_point_cloud(data_occupancy_grid_);
    reference_og_cloud_ptr =
      occupancy_grid_to_2d_point_cloud(reference_occupancy_grid_);
    if (data_pcl_cloud_.points.size() == 0 ||
        reference_pcl_cloud_.points.size() == 0) {
      return;
    }
    // if (data_og_cloud_ptr->points.size() <
    //     0.5 * reference_pcl_cloud_.points.size()) {
    //   return;
    // }
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Parameters Parameters;
    DP data_dp;
    DP reference_dp;
    PM::TransformationParameters point_cloud_transform;

    sensor_msgs::msg::PointCloud2 data_pcl_cloud_msg;
    sensor_msgs::msg::PointCloud2 reference_pcl_cloud_msg;
    pcl::toROSMsg(*data_og_cloud_ptr, data_pcl_cloud_msg);
    pcl::toROSMsg(*reference_og_cloud_ptr, reference_pcl_cloud_msg);

    data_pcl_cloud_msg.header.frame_id = "point_cloud";
    data_pcl_cloud_msg.header.stamp = get_clock()->now();
    reference_pcl_cloud_msg.header.frame_id = "point_cloud";
    reference_pcl_cloud_msg.header.stamp = get_clock()->now();

    data_dp =
      PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(data_pcl_cloud_msg);

    reference_dp = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(
      reference_pcl_cloud_msg);

    PM::ICP icp;
    icp.setDefault();

    point_cloud_transform = icp(data_dp, reference_dp);

    point_cloud_transform_.translation.x = point_cloud_transform(0, 3);
    point_cloud_transform_.translation.y = point_cloud_transform(1, 3);
    point_cloud_transform_.translation.z = point_cloud_transform(2, 3);
    tf2::Matrix3x3 tf_rot(
      point_cloud_transform(0, 0), point_cloud_transform(0, 1),
      point_cloud_transform(0, 2), point_cloud_transform(1, 0),
      point_cloud_transform(1, 1), point_cloud_transform(1, 2),
      point_cloud_transform(2, 0), point_cloud_transform(2, 1),
      point_cloud_transform(2, 2));
    tf2::Quaternion tf_quat;
    tf_rot.getRotation(tf_quat);
    point_cloud_transform_.rotation.x = tf_quat.x();
    point_cloud_transform_.rotation.y = tf_quat.y();
    point_cloud_transform_.rotation.z = tf_quat.z();
    point_cloud_transform_.rotation.w = tf_quat.w();

    PM::DataPoints transformed_data(data_dp);
    icp.transformations.apply(transformed_data, point_cloud_transform);

    sensor_msgs::msg::PointCloud2 transformed_data_pcl_cloud_msg;
    transformed_data_pcl_cloud_msg =
      PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(
        transformed_data, "point_cloud", get_clock()->now());
    pcl::fromROSMsg(transformed_data_pcl_cloud_msg, transformed_pcl_cloud_);
    transformed_point_cloud_publisher_->publish(transformed_data_pcl_cloud_msg);
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr
  point_cloud_to_occupancy_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // calculate the centroid
    Eigen::Matrix<float, 4, 1> centroid;
    pcl::ConstCloudIterator<pcl::PointXYZ> cloud_iterator(*cloud);
    pcl::compute3DCentroid(cloud_iterator, centroid);

    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();

    for (const auto &point : cloud->points) {
      if (point.x > max_x) {
        max_x = point.x;
      }
      if (point.y > max_y) {
        max_y = point.y;
      }
      if (point.x < min_x) {
        min_x = point.x;
      }
      if (point.y < min_y) {
        min_y = point.y;
      }
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid =
      std::make_shared<nav_msgs::msg::OccupancyGrid>();
    cloud->width = cloud->points.size();
    occupancy_grid->header.frame_id = "transformed_map";
    occupancy_grid->header.stamp = get_clock()->now();
    occupancy_grid->info.resolution = 0.1;
    occupancy_grid->info.width =
      std::abs(max_x - min_x) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.height =
      std::abs(max_y - min_y) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.origin.position.x = min_x;
    occupancy_grid->info.origin.position.y = min_y;
    occupancy_grid->info.origin.position.z = 0;
    occupancy_grid->info.origin.orientation.x = 0;
    occupancy_grid->info.origin.orientation.y = 0;
    occupancy_grid->info.origin.orientation.z = 0;
    occupancy_grid->info.origin.orientation.w = 1;
    occupancy_grid->data.resize(
      occupancy_grid->info.width * occupancy_grid->info.height, 0);
    for (const auto &point : cloud->points) {
      int x = (point.x - min_x) / occupancy_grid->info.resolution;
      int y = (point.y - min_y) / occupancy_grid->info.resolution;
      int index = y * occupancy_grid->info.width + x;
      occupancy_grid->data.at(index) = 100;
    }
    return occupancy_grid;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr occupancy_grid_to_2d_point_cloud(
    const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    float resolution = occupancy_grid->info.resolution;
    float origin_x = occupancy_grid->info.origin.position.x;
    float origin_y = occupancy_grid->info.origin.position.y;
    int width = occupancy_grid->info.width;
    int height = occupancy_grid->info.height;

    // Iterate through each cell in the occupancy grid
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        // Calculate the index in the occupancy data array
        int index = y * width + x;

        // Check if this cell is occupied (value of 100)
        if (occupancy_grid->data[index] == 100) {
          // Calculate the real-world coordinates of this cell
          float point_x = origin_x + x * resolution;
          float point_y = origin_y + y * resolution;

          // Create a new point and add it to the point cloud
          pcl::PointXYZ point;
          point.x = point_x;
          point.y = point_y;
          point.z = 0.0; // Occupancy grid is 2D, so z is 0
          cloud->points.push_back(point);
        }
      }
    }

    // Set width and height of the point cloud
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized point cloud

    return cloud;
  }
  void timer_callback()
  {
    match_point_cloud();

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid =
      point_cloud_to_occupancy_grid(
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
          data_pcl_cloud_));

    occupancy_grid_publisher_->publish(*occupancy_grid);

    geometry_msgs::msg::TransformStamped transformed_point_cloud_tf;
    transformed_point_cloud_tf.header.stamp = get_clock()->now();
    transformed_point_cloud_tf.header.frame_id = "map";
    transformed_point_cloud_tf.child_frame_id = "transformed_map";
    transformed_point_cloud_tf.transform = point_cloud_transform_;
    tf_broadcaster->sendTransform(transformed_point_cloud_tf);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    occupancy_grid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    transformed_point_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    reference_point_cloud_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    data_point_cloud_subscriber_;

  pcl::PointCloud<pcl::PointXYZ> reference_pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZ> data_pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZ> transformed_pcl_cloud_;

  geometry_msgs::msg::Transform point_cloud_transform_;

  nav_msgs::msg::OccupancyGrid::SharedPtr data_occupancy_grid_;
  nav_msgs::msg::OccupancyGrid::SharedPtr reference_occupancy_grid_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICP>());
  rclcpp::shutdown();
  return 0;
}
