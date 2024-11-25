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
  }

private:
  bool load_clouds()
  {
    std::string object_path =
      std::string(PROJECT_PATH) + "/objects/" + objects_path_;
    std::string cloud_path =
      std::string(PROJECT_PATH) + "/clouds/" + cloud_path_;
    if (!std::filesystem::is_directory(object_path)) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Directory " << object_path << "does not exist"); return false;
    } else {
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
      }
    }
    if (!std::filesystem::is_directory(cloud_path)) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Directory " << cloud_path << "does not exist");
      return false;
    } else {
      if (pcl::io::loadPCDFile(cloud_path, full_cloud_) == -1) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error loading file " << cloud_path);
        return false;
      }
    }
    return true;
  }

  void combine_clouds()
  {
    for (const auto &cloud : clouds_) {
      combined_cloud_ += *cloud;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;

  void timer_callback()
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(combined_cloud_, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = get_clock()->now();
    point_cloud_publisher_->publish(cloud_msg);

    for (const auto &cloud : clouds_) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_msg.header.frame_id = "world";
      cloud_msg.header.stamp = get_clock()->now();
      point_cloud_publisher_->publish(cloud_msg);
    }
  }

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
