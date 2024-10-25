// #include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <string>

#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/PointMatcher_ROS.h"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Localize : public rclcpp::Node {
public:
  Localize() : Node("localize_node")
  {
    RCLCPP_INFO(get_logger(), "Localize node started");
    declare_parameter("reference_map_file", "changeme.pcd");
    reference_map_file_ = get_parameter("reference_map_file").as_string();

    pcl::io::loadPCDFile(std::string(PROJECT_PATH) + "/maps/" +
                           reference_map_file_,
                         reference_cloud_);

    reference_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    point_cloud_transform_ = std::make_shared<PM::TransformationParameters>(PM::TransformationParameters::Identity(4, 4));
    data_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    reference_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    data_msg_->header.frame_id = "orb_point_cloud";
    reference_msg_->header.frame_id = "orb_point_cloud";

    pcl::toROSMsg(reference_cloud_, *reference_msg_);

    reference_dp_ = std::make_shared<PM::DataPoints>(
      PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(*reference_msg_));

    // create subscription
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/luci/ir_left_camera", 10, std::bind(&Localize::point_cloud_callback, this, _1));

    // create publishers
    luci_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("luci_point_cloud", 10);
    orb_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("orb_point_cloud", 10);

    // set up transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // set up the timer
    timer_ =
      create_wall_timer(100ms, std::bind(&Localize::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO_STREAM(get_logger(), "timer callback");
    if (point_cloud_transform_) {
      geometry_msgs::msg::TransformStamped luci_tf;
      luci_tf.header.stamp = get_clock()->now();
      luci_tf.header.frame_id = "map";
      luci_tf.child_frame_id = "luci_point_cloud";
      luci_tf.transform.translation.x = (*point_cloud_transform_)(0, 3);
      luci_tf.transform.translation.y = (*point_cloud_transform_)(1, 3);
      luci_tf.transform.translation.z = (*point_cloud_transform_)(2, 3);
      Eigen::Quaternionf q(point_cloud_transform_->block<3, 3>(0, 0));
      luci_tf.transform.rotation.x = q.x();
      luci_tf.transform.rotation.y = q.y();
      luci_tf.transform.rotation.z = q.z();
      luci_tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(luci_tf);

      data_msg_->header.stamp = get_clock()->now();
      luci_pub_->publish(*data_msg_);
    }

    geometry_msgs::msg::TransformStamped orb_tf;
    orb_tf.header.stamp = get_clock()->now();
    orb_tf.header.frame_id = "map";
    orb_tf.child_frame_id = "orb_point_cloud";
    tf_broadcaster_->sendTransform(orb_tf);

    geometry_msgs::msg::TransformStamped test_tf;
    test_tf.header.stamp = get_clock()->now();
    test_tf.header.frame_id = "map";
    test_tf.child_frame_id = "base_camera";
    tf_broadcaster_->sendTransform(test_tf);

    reference_msg_->header.stamp = get_clock()->now();
    reference_msg_->header.frame_id = "orb_point_cloud";
    orb_pub_->publish(*reference_msg_);
  }

  void point_cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg =
      filter_point_cloud(msg);

    data_msg_ = msg;

    data_dp_ = std::make_shared<PM::DataPoints>(
      PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(*filtered_msg));

    match_point_cloud();
  }

  const sensor_msgs::msg::PointCloud2::SharedPtr
  filter_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.1);
    sor.filter(*sor_cloud);
    sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg(
      new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*sor_cloud, *filtered_msg);
    return filtered_msg;
  }

  void match_point_cloud()
  {
    PM::ICP icp;
    icp.setDefault();

    point_cloud_transform_ = std::make_shared<PM::TransformationParameters>(
      icp(*data_dp_, *reference_dp_));

    PM::DataPoints transformed_data(*data_dp_);
    icp.transformations.apply(transformed_data, *point_cloud_transform_);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr luci_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr orb_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  sensor_msgs::msg::PointCloud2::SharedPtr reference_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr data_msg_;

  std::string reference_map_file_;

  pcl::PointCloud<pcl::PointXYZ> reference_cloud_;

  typedef PointMatcher<float> PM;
  typedef PM::DataPoints dp_;
  typedef PM::Parameters Parameters;
  std::shared_ptr<PM::TransformationParameters> point_cloud_transform_;
  std::shared_ptr<PM::DataPoints> reference_dp_;
  std::shared_ptr<PM::DataPoints> data_dp_;
  std::shared_ptr<PM::Parameters> params;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localize>());
  rclcpp::shutdown();
  return 0;
}
