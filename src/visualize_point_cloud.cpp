#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/point_field__struct.hpp>
#include <string>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/common/centroid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;
// using std::placeholders::_1;

class VisualizePointCloud : public rclcpp::Node {
public:
  VisualizePointCloud() : Node("visualize_point_cloud")
  {
    // declare parameters
    declare_parameter("pcl_file", "changeme.pcd");
    declare_parameter("z_threshold", 0.1);
    pcl_file = get_parameter("pcl_file").as_string();
    z_threshold = get_parameter("z_threshold").as_double();

    // create publishers
    point_cloud2_publisher =
      create_publisher<sensor_msgs::msg::PointCloud2>("orb_point_cloud2", 10);

    // Initialize the transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer = create_wall_timer(
      500ms, std::bind(&VisualizePointCloud::timer_callback, this));

    load_point_cloud();
  }

private:
  void load_point_cloud()
  {
    RCLCPP_INFO_STREAM(get_logger(),
                       "loading point cloud from "
                         << std::string(PROJECT_PATH) + "/maps/" + pcl_file);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::io::loadPCDFile(std::string(PROJECT_PATH) + "/maps/" + pcl_file,
                         pcl_cloud);

    RCLCPP_INFO_STREAM(get_logger(),
                       "loaded " << pcl_cloud.points.size() << " points");

    // now i can filter the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

    cloud_filtered = cloud_ptr;
    // voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered);

    // statistical outlier removal
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // RCLCPP_INFO_STREAM(get_logger(), "original point cloud size: "
    //                                    << cloud_filtered->points.size());
    // sor.setInputCloud(cloud_filtered);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(0.01);
    // sor.filter(*cloud_filtered);
    // RCLCPP_INFO_STREAM(get_logger(), "filtered point cloud size: "
    //                                    << cloud_filtered->points.size());

    // convex hull
    // pcl::ConvexHull<pcl::PointXYZ> chull;
    // chull.setInputCloud(cloud_filtered);
    // chull.reconstruct(*cloud_filtered);

    // pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    // condrem.setInputCloud(cloud_filtered);
    //
    // pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(
    //   new pcl::ConditionAnd<pcl::PointXYZ>());
    //
    // condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
    //   new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT,
    //   z_threshold)));
    //
    // condrem.setCondition(condition);
    // condrem.setKeepOrganized(true);
    // condrem.filter(*cloud_filtered);

    // pcl::VoxelGrid<pcl::PointXYZ> avg;
    // avg.setInputCloud(cloud_ptr);
    // avg.setLeafSize(0.25f, 0.25f, 0.25f);
    // avg.filter(*cloud_filtered);
    //
    // //searchPoint
    // pcl::PointXYZ searchPoint = cloud_filtered->at(0) ;
    //
    // //result from radiusSearch()
    // std::vector<int> pointIdxRadiusSearch;
    // std::vector<float> pointRadiusSquaredDistance;
    //
    // //kdTree
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud (cloud_filtered);
    // kdtree.setSortedResults(true);
    //
    // if ( kdtree.radiusSearch (searchPoint, 100, pointIdxRadiusSearch,
    // pointRadiusSquaredDistance) > 0 )
    // {
    //     //delete every point in target
    //     for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
    //     {
    //         //is this the way to erase correctly???
    //         //
    //         cloud_out.push_back(cloudFiltered->points[pointIdxRadiusSearch[j]]);
    //         cloud_filtered->erase(cloud_filtered->begin() +
    //         pointIdxRadiusSearch[j]);
    //     }
    // }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // Adjust proximity threshold (in meters)
    ec.setMinClusterSize(100);    // Minimum number of points in a cluster
    ec.setMaxClusterSize(50000);  // Maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    // Create a point cloud to hold the combined clusters
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_clusters(
      new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &cluster : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

      for (const auto &idx : cluster.indices) {
        cluster_cloud->points.push_back(cloud_filtered->points[idx]);
      }

      cluster_cloud->width = cluster_cloud->points.size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;

      // Optionally: you can analyze or process each cluster here (e.g., compute
      // centroid)
      // Eigen::Vector4f centroid;
      // pcl::compute3DCentroid(*cluster_cloud, centroid);
      // std::cout << "Cluster centroid: " << centroid.transpose() << std::endl;

      // Combine each cluster into the final point cloud
      *final_clusters += *cluster_cloud;

    }
    // moving least square
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(final_clusters);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree2);
    mls.setSearchRadius(0.1);
    mls.setUpsamplingMethod(
      pcl::MovingLeastSquares<pcl::PointXYZ,
                              pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
    mls.setUpsamplingRadius(0.05);
    mls.setUpsamplingStepSize(0.02);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(
      new pcl::PointCloud<pcl::PointXYZ>);
    mls.process(*cloud_smoothed);

    pcl::toROSMsg(*cloud_smoothed, point_cloud2);
  }
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "point_cloud";

    t.transform.translation.z = 2;
    tf_broadcaster->sendTransform(t);

    point_cloud2.header.stamp = get_clock()->now();
    point_cloud2.header.frame_id = "point_cloud";
    point_cloud2_publisher->publish(point_cloud2);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud2_publisher;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    occupancy_grid_sub;

  sensor_msgs::msg::PointCloud2 point_cloud2;

  tf2::Quaternion initial_orientation;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  std::string pcl_file;
  double z_threshold;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizePointCloud>());
  rclcpp::shutdown();
  return 0;
}
