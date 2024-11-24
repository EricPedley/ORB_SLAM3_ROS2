#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class Visualize : public rclcpp::Node {
public:
  Visualize() : Node("visualize")
  {
    // declare parameters
    declare_parameter("objects_path", "");

    // get parameters
    get_parameter("objects_path", objects_path_);

    if (!load_clouds()) {
      RCLCPP_ERROR(get_logger(), "Error loading clouds");
      rclcpp::shutdown();
    }
  }

private:
  bool load_clouds()
  {
    std::string path =
      std::string(PROJECT_PATH) + "/semantic_maps/" + objects_path_;
    if (!std::filesystem::is_directory(path)) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Directory " << path << "does not exist");
      return false;
    } else {
      for (const auto &file : std::filesystem::directory_iterator(path)) {
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
    return true;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
  std::string objects_path_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualize>());
  rclcpp::shutdown();
  return 0;
}
