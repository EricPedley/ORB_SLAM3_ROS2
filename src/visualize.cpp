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
  Visualize() : Node("visualize"), iterator(0)
  {
    // declare parameters
    declare_parameter("database_output_name", "");

    // get parameters
    get_parameter("database_output_name", database_output_name_);

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
    std::string output_path =
      std::string(PROJECT_PATH) + "/database_outputs/" + database_output_name_;
    if (!std::filesystem::is_directory(output_path)) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Directory " << output_path << " does not exist");
      return false;
    } else {
      int objects = 0;
      std::string objects_path = output_path + "/objects/";
      for (const auto &file :
           std::filesystem::directory_iterator(objects_path)) {
        if (file.is_regular_file() && file.path().extension() == ".pcd") {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
          if (pcl::io::loadPCDFile(file.path().string(), *cloud) == -1) {
            RCLCPP_ERROR_STREAM(get_logger(), "Error loading file " << file);
            return false;
          } else {
            RCLCPP_INFO_STREAM(get_logger(), "Loaded object with "
                                               << cloud->size() << " points");
            RCLCPP_INFO_STREAM(get_logger(),
                               "file name: " << file.path().string());
            RCLCPP_INFO_STREAM(get_logger(),
                               "point color: "
                                 << std::to_string(cloud->points[0].r) << " "
                                 << std::to_string(cloud->points[0].g) << " "
                                 << std::to_string(cloud->points[0].b));
            RCLCPP_INFO_STREAM(get_logger(),
                               "next point color: "
                                 << std::to_string(cloud->points[1].r) << " "
                                 << std::to_string(cloud->points[1].g) << " "
                                 << std::to_string(cloud->points[1].b));
            clouds_.push_back(cloud);
          }
        }
        objects++;
      }
      RCLCPP_INFO_STREAM(get_logger(), "Loaded " << objects << " objects");
    }
    std::string cloud_path =
      output_path + "/cloud/" + database_output_name_ + ".pcd";
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
    // sensor_msgs::msg::PointCloud2 full_cloud_msg;
    // pcl::toROSMsg(combined_cloud_, full_cloud_msg);
    // full_cloud_msg.header.frame_id = "map";
    // full_cloud_msg.header.stamp = get_clock()->now();
    // full_cloud_publisher_->publish(full_cloud_msg);

    sensor_msgs::msg::PointCloud2 object_cloud_msg;
    pcl::toROSMsg(*clouds_.at(iterator), object_cloud_msg);
    object_cloud_msg.header.frame_id = "map";
    object_cloud_msg.header.stamp = get_clock()->now();
    object_cloud_publisher_->publish(object_cloud_msg);

    iterator++;
    iterator = iterator == clouds_.size() ? 0 : iterator;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    full_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    object_cloud_publisher_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
  pcl::PointCloud<pcl::PointXYZRGB> combined_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB> full_cloud_;
  std::string database_output_name_;
  std::string cloud_path_;
  int iterator;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualize>());
  rclcpp::shutdown();
  return 0;
}
