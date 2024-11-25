#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class Visualize : public rclcpp::Node {
public:
  Visualize() : Node("visualize")
  {
    // declare parameters
    declare_parameter("objects_path", "");
    declare_parameter("cloud_path", "");

    // get parameters
    get_parameter("objects_path", objects_path_);
    get_parameter("cloud_path", cloud_path_);

    if (!load_clouds()) {
      RCLCPP_ERROR(get_logger(), "Error loading clouds");
      rclcpp::shutdown();
    }

    combine_clouds();

    // define publishers
    full_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("full_cloud", 10);
    object_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("object_clouds", 10);

    // define timer
    timer_ =
      create_wall_timer(1000ms, std::bind(&Visualize::timer_callback, this));
  }

private:
  bool load_clouds()
  {
    std::string object_path =
      std::string(PROJECT_PATH) + "/objects/" + objects_path_;
    std::string cloud_path =
      std::string(PROJECT_PATH) + "/clouds/" + cloud_path_ + ".pcd";
    if (!std::filesystem::is_directory(object_path)) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Directory " << object_path << " does not exist");
      return false;
    } else {
      int objects = 0;
      for (const auto &file :
           std::filesystem::directory_iterator(object_path)) {
        if (file.is_regular_file() && file.path().extension() == ".pcd") {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
          if (pcl::io::loadPCDFile(file.path().string(), *cloud) == -1) {
            RCLCPP_ERROR_STREAM(get_logger(), "Error loading file " << file);
            return false;
          } else {
            clouds_.push_back(cloud);
          }
        }
        objects++;
      }
      RCLCPP_INFO_STREAM(get_logger(), "Loaded " << objects << " objects");
    }
    if (pcl::io::loadPCDFile(cloud_path, full_cloud_) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error loading file " << cloud_path);
      return false;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Loaded full cloud with "
                                       << full_cloud_.size() << " points");
    return true;
  }

  void combine_clouds()
  {
    for (const auto &cloud : clouds_) {
      combined_cloud_ += *cloud;
    }
  }
  void timer_callback()
  {
    sensor_msgs::msg::PointCloud2 full_cloud_msg;
    pcl::toROSMsg(combined_cloud_, full_cloud_msg);
    full_cloud_msg.header.frame_id = "map";
    full_cloud_msg.header.stamp = get_clock()->now();
    full_cloud_publisher_->publish(full_cloud_msg);

    sensor_msgs::msg::PointCloud2 object_cloud_msg;
    pcl::toROSMsg(combined_cloud_, object_cloud_msg);
    object_cloud_msg.header.frame_id = "map";
    object_cloud_msg.header.stamp = get_clock()->now();
    object_cloud_publisher_->publish(object_cloud_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    full_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    object_cloud_publisher_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
  pcl::PointCloud<pcl::PointXYZRGB> combined_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB> full_cloud_;
  std::string objects_path_;
  std::string cloud_path_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualize>());
  rclcpp::shutdown();
  return 0;
}
