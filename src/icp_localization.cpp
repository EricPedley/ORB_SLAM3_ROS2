#include <functional>
#include <memory>
#include <string>

#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/PointMatcher_ROS.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ICP : public rclcpp::Node {
public:
  ICP()
    : Node("icp_node"), init_translation_("0,0,0"),
      init_rotation_("1,0,0;0,1,0;0,0,1")
  {
    // declare parameters
    declare_parameter("reference_map_file", "changeme.pcd");

    // get parameters
    reference_map_file = get_parameter("reference_map_file").as_string();
    reference_map_file =
      std::string(PROJECT_PATH) + "/maps/" + reference_map_file;

    // load reference point cloud
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(reference_map_file, tmp_cloud) ==
        -1) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Couldn't read file " << reference_map_file);
    }

    // filter point cloud
    reference_pcl_cloud_ = filter_point_cloud(
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(tmp_cloud));

    // convert point cloud to occupancy grid
    reference_occupancy_grid_ = point_cloud_to_occupancy_grid(
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(reference_pcl_cloud_),
      "reference_map");
    reference_occupancy_grid_->header.frame_id = "reference_map";

    // create publishers
    transformed_occupancy_grid_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(
        "transformed_occupancy_grid", 10);
    reference_occupancy_grid_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("reference_occupancy_grid",
                                                     10);

    // create subscriptions
    live_occupancy_grid_subscriber_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
        "live_occupancy_grid", 10,
        std::bind(&ICP::live_occupancy_grid_callback, this, _1));

    // create tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create timer
    timer_ = create_wall_timer(1000ms, std::bind(&ICP::timer_callback, this));

    // initialize variables
    live_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

private:
  pcl::PointCloud<pcl::PointXYZ>
  filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.1);
    sor.filter(*sor_cloud);

    // radius outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
    radius_outlier.setInputCloud(sor_cloud);
    radius_outlier.setRadiusSearch(
      0.1); // Adjust based on spacing in the point cloud
    radius_outlier.setMinNeighborsInRadius(
      5); // Increase for more aggressive outlier removal
    radius_outlier.filter(*radius_cloud);
    return *radius_cloud;
  }

  typedef PointMatcher<float> PM;
  PM::TransformationParameters parseTranslation(std::string &translation,
                                                const int cloudDimension)
  {
    PM::TransformationParameters parsedTranslation;
    parsedTranslation = PM::TransformationParameters::Identity(
      cloudDimension + 1, cloudDimension + 1);

    translation.erase(std::remove(translation.begin(), translation.end(), '['),
                      translation.end());
    translation.erase(std::remove(translation.begin(), translation.end(), ']'),
                      translation.end());
    std::replace(translation.begin(), translation.end(), ',', ' ');
    std::replace(translation.begin(), translation.end(), ';', ' ');

    float translationValues[3] = {0};
    std::stringstream translationStringStream(translation);
    for (int i = 0; i < cloudDimension; i++) {
      if (!(translationStringStream >> translationValues[i])) {
        std::cerr << "An error occured while trying to parse the initial "
                  << "translation." << std::endl
                  << "No initial translation will be used" << std::endl;
        return parsedTranslation;
      }
    }
    float extraOutput = 0;
    if ((translationStringStream >> extraOutput)) {
      std::cerr << "Wrong initial translation size" << std::endl
                << "No initial translation will be used" << std::endl;
      return parsedTranslation;
    }

    for (int i = 0; i < cloudDimension; i++) {
      parsedTranslation(i, cloudDimension) = translationValues[i];
    }

    return parsedTranslation;
  }

  PM::TransformationParameters parseRotation(std::string &rotation,
                                             const int cloudDimension)
  {
    PM::TransformationParameters parsedRotation;
    parsedRotation = PM::TransformationParameters::Identity(cloudDimension + 1,
                                                            cloudDimension + 1);

    rotation.erase(std::remove(rotation.begin(), rotation.end(), '['),
                   rotation.end());
    rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'),
                   rotation.end());
    std::replace(rotation.begin(), rotation.end(), ',', ' ');
    std::replace(rotation.begin(), rotation.end(), ';', ' ');

    float rotationMatrix[9] = {0};
    std::stringstream rotationStringStream(rotation);
    for (int i = 0; i < cloudDimension * cloudDimension; i++) {
      if (!(rotationStringStream >> rotationMatrix[i])) {
        std::cerr << "An error occured while trying to parse the initial "
                  << "rotation." << std::endl
                  << "No initial rotation will be used" << std::endl;
        return parsedRotation;
      }
    }
    float extraOutput = 0;
    if ((rotationStringStream >> extraOutput)) {
      std::cerr << "Wrong initial rotation size" << std::endl
                << "No initial rotation will be used" << std::endl;
      return parsedRotation;
    }

    for (int i = 0; i < cloudDimension * cloudDimension; i++) {
      parsedRotation(i / cloudDimension, i % cloudDimension) =
        rotationMatrix[i];
    }

    return parsedRotation;
  }

  void live_occupancy_grid_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    live_occupancy_grid_ = msg;
    live_occupancy_grid_->header.frame_id = "transformed_map";
  }

  bool
  match_point_cloud(geometry_msgs::msg::TransformStamped::SharedPtr transform)
  {
    pcl::PointCloud<pcl::PointXY>::Ptr live_og_cloud_ptr =
      occupancy_grid_to_2d_point_cloud(live_occupancy_grid_);
    pcl::PointCloud<pcl::PointXY>::Ptr reference_og_cloud_ptr =
      occupancy_grid_to_2d_point_cloud(reference_occupancy_grid_);
    if (live_og_cloud_ptr->points.size() == 0 ||
        reference_og_cloud_ptr->points.size() == 0) {
      RCLCPP_ERROR(get_logger(), "No point cloud data available");
      return false;
    }

    sensor_msgs::msg::PointCloud2 live_pcl_cloud_msg;
    sensor_msgs::msg::PointCloud2 reference_pcl_cloud_msg;
    pcl::toROSMsg(*live_og_cloud_ptr, live_pcl_cloud_msg);
    pcl::toROSMsg(*reference_og_cloud_ptr, reference_pcl_cloud_msg);

    live_dp = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(
      live_pcl_cloud_msg, true);

    reference_dp = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(
      reference_pcl_cloud_msg, true);

    std::string config_file =
      std::string(PROJECT_PATH) + "/config/pointmatcher_config.yaml";
    std::ifstream ifs(config_file.c_str());
    if (!ifs.good()) {
      RCLCPP_ERROR(get_logger(), "Cannot open config file %s",
                   config_file.c_str());
      return false;
    }
    // icp.loadFromYaml(ifs);
    icp.setDefault();

    int cloud_dimension = reference_dp.getEuclideanDim();

    PM::TransformationParameters translation =
      parseTranslation(init_translation_, cloud_dimension);
    PM::TransformationParameters rotation =
      parseRotation(init_rotation_, cloud_dimension);
    PM::TransformationParameters initTransfo = translation * rotation;

    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(initTransfo)) {
      RCLCPP_ERROR(
        get_logger(),
        "\nInitial transformation is not rigid, identiy will be used\n");
      initTransfo = PM::TransformationParameters::Identity(cloud_dimension + 1,
                                                           cloud_dimension + 1);
    }

    const DP initializedData = rigidTrans->compute(live_dp, initTransfo);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(initializedData, reference_dp);

    // Transform data to express it in ref
    DP live_out(initializedData);
    icp.transformations.apply(live_out, T);

    point_cloud_transform = T;

    transform->transform.translation.x = point_cloud_transform(0, 2);
    transform->transform.translation.y = point_cloud_transform(1, 2);
    transform->transform.translation.z = 0.0;
    tf2::Matrix3x3 tf_rot(point_cloud_transform(0, 0),
                          point_cloud_transform(0, 1), 0.0,
                          point_cloud_transform(1, 0),
                          point_cloud_transform(1, 1), 0.0, 0.0, 0.0, 1.0);
    tf2::Quaternion tf_quat;
    tf_rot.getRotation(tf_quat);
    transform->transform.rotation.x = tf_quat.x();
    transform->transform.rotation.y = tf_quat.y();
    transform->transform.rotation.z = tf_quat.z();
    transform->transform.rotation.w = tf_quat.w();

    // sensor_msgs::msg::PointCloud2 transformed_pcl_cloud_msg;
    // transformed_pcl_cloud_msg =
    //   PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(
    //     live_out, "transformed_map", get_clock()->now());
    // pcl::fromROSMsg(transformed_pcl_cloud_msg, transformed_pcl_cloud_);
    return true;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr
  point_cloud_to_occupancy_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                std::string frame)
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
    occupancy_grid->header.frame_id = frame;
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

  pcl::PointCloud<pcl::PointXY>::Ptr occupancy_grid_to_2d_point_cloud(
    const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid)
  {
    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
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
          pcl::PointXY point;
          point.x = point_x;
          point.y = point_y;
          // point.z = 0.0; // Occupancy grid is 2D, so z is 0
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
    nav_msgs::msg::OccupancyGrid::SharedPtr transformed_occupancy_grid(
      new nav_msgs::msg::OccupancyGrid);

    geometry_msgs::msg::TransformStamped::SharedPtr transformed_point_cloud_tf =
      std::make_shared<geometry_msgs::msg::TransformStamped>();
    bool success = match_point_cloud(transformed_point_cloud_tf);
    if (!success) {
      return;
    }
    transformed_point_cloud_tf->header.stamp = get_clock()->now();
    transformed_point_cloud_tf->header.frame_id = "map";
    transformed_point_cloud_tf->child_frame_id = "transformed_map";
    tf_broadcaster->sendTransform(*transformed_point_cloud_tf);

    geometry_msgs::msg::TransformStamped reference_point_cloud_tf;
    reference_point_cloud_tf.header.stamp = get_clock()->now();
    reference_point_cloud_tf.header.frame_id = "map";
    reference_point_cloud_tf.child_frame_id = "reference_map";
    tf_broadcaster->sendTransform(reference_point_cloud_tf);

    transformed_occupancy_grid->header.stamp = get_clock()->now();
    transformed_occupancy_grid->header.frame_id = "transformed_map";
    transformed_occupancy_grid_publisher_->publish(*live_occupancy_grid_);

    reference_occupancy_grid_->header.stamp = get_clock()->now();
    reference_occupancy_grid_publisher_->publish(*reference_occupancy_grid_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    transformed_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    reference_occupancy_grid_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    live_occupancy_grid_subscriber_;

  pcl::PointCloud<pcl::PointXYZ> reference_pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZ> live_pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZ> transformed_pcl_cloud_;

  geometry_msgs::msg::Transform point_cloud_transform_;

  nav_msgs::msg::OccupancyGrid::SharedPtr live_occupancy_grid_;
  nav_msgs::msg::OccupancyGrid::SharedPtr reference_occupancy_grid_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  typedef PM::DataPoints DP;
  typedef PM::Parameters Parameters;
  PM::ICP icp;
  DP live_dp;
  DP reference_dp;
  PM::TransformationParameters point_cloud_transform;
  std::string init_translation_;
  std::string init_rotation_;

  std::string reference_map_file;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICP>());
  rclcpp::shutdown();
  return 0;
}
