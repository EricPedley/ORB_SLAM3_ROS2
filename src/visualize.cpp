#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <filesystem>

#include <rclcpp/rclcpp.hpp>

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
    declare_parameter("output_file_name", "");

    // get parameters
    get_parameter("output_file_name", output_file_name_);

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
    box_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "bounding_boxes", 10);
    label_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "labels", 10);

    // define timer
    timer_ =
      create_wall_timer(1000ms, std::bind(&Visualize::timer_callback, this));
  }

private:
  bool load_clouds()
  {
    std::string output_path =
      std::string(PROJECT_PATH) + "/output/" + output_file_name_;
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
            std::string file_name = file.path().stem();
            clouds_.push_back({file_name, cloud});
          }
        }
        objects++;
      }
      RCLCPP_INFO_STREAM(get_logger(), "Loaded " << objects << " objects");
    }
    std::string cloud_path =
      output_path + "/cloud/" + output_file_name_ + ".pcd";
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
      combined_cloud_ += *cloud.second;
      bounding_boxes_.push_back(calculate_box(*cloud.second));
    }
  }

  std::vector<geometry_msgs::msg::Point>
  calculate_box(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
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

    geometry_msgs::msg::Point top_right_back;
    top_right_back.x = box.x_max;
    top_right_back.y = box.y_max;
    top_right_back.z = box.z_max;

    geometry_msgs::msg::Point top_left_back;
    top_left_back.x = box.x_min;
    top_left_back.y = box.y_max;
    top_left_back.z = box.z_max;

    geometry_msgs::msg::Point top_left_front;
    top_left_front.x = box.x_min;
    top_left_front.y = box.y_min;
    top_left_front.z = box.z_max;

    geometry_msgs::msg::Point bottom_left_front;
    bottom_left_front.x = box.x_min;
    bottom_left_front.y = box.y_min;
    bottom_left_front.z = box.z_min;

    geometry_msgs::msg::Point bottom_right_front;
    bottom_right_front.x = box.x_max;
    bottom_right_front.y = box.y_min;
    bottom_right_front.z = box.z_min;

    geometry_msgs::msg::Point bottom_right_back;
    bottom_right_back.x = box.x_max;
    bottom_right_back.y = box.y_max;
    bottom_right_back.z = box.z_min;

    geometry_msgs::msg::Point bottom_left_back;
    bottom_left_back.x = box.x_min;
    bottom_left_back.y = box.y_max;
    bottom_left_back.z = box.z_min;

    points.push_back(top_right_front);
    points.push_back(top_right_back);
    points.push_back(top_right_back);
    points.push_back(top_left_back);
    points.push_back(top_left_back);
    points.push_back(top_left_front);
    points.push_back(top_left_front);
    points.push_back(top_right_front);
    points.push_back(top_right_front);
    points.push_back(bottom_right_front);
    points.push_back(bottom_right_front);
    points.push_back(bottom_right_back);
    points.push_back(bottom_right_back);
    points.push_back(bottom_left_back);
    points.push_back(bottom_left_back);
    points.push_back(bottom_left_front);
    points.push_back(bottom_left_front);
    points.push_back(bottom_right_front);
    points.push_back(top_right_back);
    points.push_back(bottom_right_back);
    points.push_back(top_left_back);
    points.push_back(bottom_left_back);
    points.push_back(top_left_front);
    points.push_back(bottom_left_front);

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
    pcl::toROSMsg(combined_cloud_, object_cloud_msg);
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
      box_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      box_marker.scale.x = 0.01;
      box_marker.color.r = 1.0;
      box_marker.color.a = 1.0;
      box_markers.markers.push_back(box_marker);
    }
    box_publisher_->publish(box_markers);

    int iter = 0;
    visualization_msgs::msg::MarkerArray label_markers;
    for (const auto &cloud : clouds_) {
      // calculate the centroid
      pcl::PointXYZRGB centroid;
      for (const auto &point : *cloud.second) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
      }
      centroid.x /= cloud.second->size();
      centroid.y /= cloud.second->size();
      centroid.z /= cloud.second->size();

      std::string label = cloud.first;
      visualization_msgs::msg::Marker label_marker;
      label_marker.header.frame_id = "map";
      label_marker.header.stamp = get_clock()->now();
      label_marker.id = iter;
      label_marker.action = visualization_msgs::msg::Marker::ADD;
      label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label_marker.text = label;
      label_marker.pose.position.x = centroid.x;
      label_marker.pose.position.y = centroid.y;
      label_marker.pose.position.z = centroid.z;
      label_marker.scale.z = 0.25;
      label_marker.color.r = 1.0;
      label_marker.color.g = 1.0;
      label_marker.color.b = 1.0;
      label_marker.color.a = 1.0;
      label_markers.markers.push_back(label_marker);

      iter++;
    }
    label_publisher_->publish(label_markers);

    iterator++;
    iterator = iterator == clouds_.size() ? 0 : iterator;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    full_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    object_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    box_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    label_publisher_;

  std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
    clouds_;
  std::vector<std::vector<geometry_msgs::msg::Point>> bounding_boxes_;
  pcl::PointCloud<pcl::PointXYZRGB> combined_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB> full_cloud_;
  std::string output_file_name_;
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
