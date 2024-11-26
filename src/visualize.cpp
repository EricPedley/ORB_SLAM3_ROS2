#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <filesystem>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

struct Box {
  float x_min = std::numeric_limits<float>::infinity();
  float x_max = -std::numeric_limits<float>::infinity();
  float y_min = std::numeric_limits<float>::infinity();
  float y_max = -std::numeric_limits<float>::infinity();
  float z_min = std::numeric_limits<float>::infinity();
  float z_max = -std::numeric_limits<float>::infinity();
};

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
      bounding_boxes_.push_back(calculate_box(*cloud));
    }
  }

  std::vector<geometry_msgs::msg::Point> calculate_box(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
  {
    Box box;
    for (const auto &point : cloud) {
      box.x_max = point.x > box.x_max ? point.x : box.x_max;
      box.x_min = point.x < box.x_min ? point.x : box.x_min;
      box.y_max = point.y > box.y_max ? point.y : box.y_max;
      box.y_min = point.y < box.y_min ? point.y : box.y_min;
      box.z_max = point.z > box.z_max ? point.z : box.z_max;
      box.z_min = point.z < box.z_min ? point.z : box.z_min;
    }

    std::vector<geometry_msgs::msg::Point> points;
    geometry_msgs::msg::Point top_right_front;
    top_right_front.x = box.x_max;
    top_right_front.y = box.y_min;
    top_right_front.z = box.z_max;
    points.push_back(top_right_front);

    geometry_msgs::msg::Point top_right_back;
    top_right_back.x = box.x_max;
    top_right_back.y = box.y_max;
    top_right_back.z = box.z_max;
    points.push_back(top_right_back);

    geometry_msgs::msg::Point top_left_back;
    top_left_back.x = box.x_min;
    top_left_back.y = box.y_max;
    top_left_back.z = box.z_max;
    points.push_back(top_left_back);

    geometry_msgs::msg::Point top_left_front;
    top_left_front.x = box.x_min;
    top_left_front.y = box.y_min;
    top_left_front.z = box.z_max;
    points.push_back(top_left_front);

    geometry_msgs::msg::Point bottom_left_front;
    bottom_left_front.x = box.x_min;
    bottom_left_front.y = box.y_min;
    bottom_left_front.z = box.z_min;
    points.push_back(bottom_left_front);

    geometry_msgs::msg::Point bottom_right_front;
    bottom_right_front.x = box.x_max;
    bottom_right_front.y = box.y_min;
    bottom_right_front.z = box.z_min;
    points.push_back(bottom_right_front);

    geometry_msgs::msg::Point bottom_right_back;
    bottom_right_back.x = box.x_max;
    bottom_right_back.y = box.y_max;
    bottom_right_back.z = box.z_min;
    points.push_back(bottom_right_back);

    geometry_msgs::msg::Point bottom_left_back;
    bottom_left_back.x = box.x_min;
    bottom_left_back.y = box.y_max;
    bottom_left_back.z = box.z_min;
    points.push_back(bottom_left_back);

    return points;
  }

  void timer_callback()
  {
    sensor_msgs::msg::PointCloud2 full_cloud_msg;
    pcl::toROSMsg(full_cloud_, full_cloud_msg);
    full_cloud_msg.header.frame_id = "map";
    full_cloud_msg.header.stamp = get_clock()->now();
    full_cloud_publisher_->publish(full_cloud_msg);

    sensor_msgs::msg::PointCloud2 object_cloud_msg;
    pcl::toROSMsg(*clouds_.at(iterator), object_cloud_msg);
    object_cloud_msg.header.frame_id = "map";
    object_cloud_msg.header.stamp = get_clock()->now();
    object_cloud_publisher_->publish(object_cloud_msg);

    visualization_msgs::msg::MarkerArray box_markers;
    for (size_t i = 0; i < bounding_boxes_.size(); i++) {
      visualization_msgs::msg::Marker box_marker;
      box_marker.header.frame_id = "map";
      box_marker.header.stamp = get_clock()->now();
      box_marker.id = i;
      box_marker.action = visualization_msgs::msg::Marker::ADD;
      box_marker.points = bounding_boxes_.at(i);
      box_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    }

    iterator++;
    iterator = iterator == clouds_.size() ? 0 : iterator;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    full_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    object_cloud_publisher_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
  std::vector<std::vector<geometry_msgs::msg::Point>> bounding_boxes_;
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
