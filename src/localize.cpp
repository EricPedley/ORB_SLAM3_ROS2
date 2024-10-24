#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "pointmatcher/PointMatcher.h"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Localize : public rclcpp::Node {
public:
  Localize() : Node("localize")
  {
    RCLCPP_INFO(get_logger(), "Localize node started");
    declare_parameter("reference_map_file", "changeme.pcd");
    reference_map_file_ = get_parameter("reference_map_file").as_string();

    pcl::io::loadPCDFile(std::string(PROJECT_PATH) + "/maps/" + reference_map_file_,
                         reference_cloud_);

    // dp = std::make_shared<PM::DataPoints>(
    //   DP::load(reference_map_file_));

    // dp = std::make_shared<PM::DataPoints>(
    //     )

    // create subscription
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", 10, std::bind(&Localize::point_cloud_callback, this, _1));

    // RCLCPP_INFO_STREAM(get_logger(), "Loaded reference map with " << dp->features.cols() << " points");
  }

private:
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM(get_logger(), "got point cloud msg: " << msg->header.stamp.sec << " " << msg->header.stamp.nanosec);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_sub_;

  std::string reference_map_file_;

  pcl::PointCloud<pcl::PointXYZ> reference_cloud_;

  typedef PointMatcher<float> PM;
  typedef PM::DataPoints DP;
  typedef PM::Parameters Parameters;
  std::shared_ptr<PM::DataPoints> dp;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localize>());
  rclcpp::shutdown();
  return 0;
}
