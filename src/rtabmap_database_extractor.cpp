#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/utilite/UTimer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>


#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RTABMapDatabaseExtractor : public rclcpp::Node {
public:
  RTABMapDatabaseExtractor() : Node("rtabmap_database_extractor")
  {
    declare_parameter("rtabmap_db", "");
    std::string rtabmap_database = get_parameter("rtabmap_db").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "rtabmap_db: " << rtabmap_database);
    rtabmap_database_path_ =
      std::string(PROJECT_PATH) + "/maps/" + rtabmap_database;
    RCLCPP_INFO_STREAM(get_logger(), "rtabmap_db: " << rtabmap_database_path_);

    accumulated_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    load_rtabmap_db(rtabmap_database_path_);

    // create publisher
    point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

    // create timer
    timer_ = create_wall_timer(
      500ms, std::bind(&RTABMapDatabaseExtractor::timer_callback, this));
  }

private:
  void timer_callback()
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*accumulated_cloud, pcl_pc2);
    sensor_msgs::msg::PointCloud2 msg;
    pcl_conversions::fromPCL(pcl_pc2, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = get_clock()->now();
    point_cloud_publisher_->publish(msg);
  }
  bool load_rtabmap_db(const std::string &db_path)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    rtabmap::ParametersMap parameters;
    rtabmap::DBDriver *driver = rtabmap::DBDriver::create();

    if (driver->openConnection(db_path)) {
      parameters = driver->getLastParameters();
      driver->closeConnection(false);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open database");
      return false;
    }
    delete driver;
    driver = 0;

    UTimer timer;

    RCLCPP_INFO_STREAM(get_logger(), "Loading database: " << db_path);
    rtabmap::Rtabmap rtabmap;
    rtabmap.init(parameters, db_path);
    RCLCPP_INFO_STREAM(get_logger(),
                       "Loaded database in " << timer.ticks() << "s");

    std::map<int, rtabmap::Signature> nodes;
    std::map<int, rtabmap::Transform> optimizedPoses;
    std::multimap<int, rtabmap::Link> links;
    RCLCPP_INFO(get_logger(), "Optimizing the map...");
    rtabmap.getGraph(optimizedPoses, links, true, true, &nodes, true, true,
                     true, true);
    printf("Optimizing the map... done (%fs, poses=%d).\n", timer.ticks(),
           (int)optimizedPoses.size());

    RCLCPP_INFO_STREAM(get_logger(), "Optimizing the map... done ("
                                       << timer.ticks() << "s, poses="
                                       << optimizedPoses.size() << ").");

    if (optimizedPoses.size() == 0) {
      RCLCPP_ERROR(get_logger(), "No optimized poses found");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr assembledCloudI(
      new pcl::PointCloud<pcl::PointXYZI>);
    std::map<int, rtabmap::Transform> robotPoses;
    std::vector<std::map<int, rtabmap::Transform>> cameraPoses;
    std::map<int, rtabmap::Transform> scanPoses;
    std::map<int, double> cameraStamps;
    std::map<int, std::vector<rtabmap::CameraModel>> cameraModels;
    std::map<int, cv::Mat> cameraDepths;
    std::vector<int> rawViewpointIndices;
    std::map<int, rtabmap::Transform> rawViewpoints /*  */;
    for (std::map<int, rtabmap::Transform>::iterator iter =
           optimizedPoses.lower_bound(1);
         iter != optimizedPoses.end(); ++iter) {
      rtabmap::Signature node = nodes.find(iter->first)->second;

      // uncompress data
      std::vector<rtabmap::CameraModel> models =
        node.sensorData().cameraModels();
      cv::Mat rgb;
      cv::Mat depth;

      pcl::IndicesPtr indices(new std::vector<int>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI;
      if (node.getWeight() != -1) {
        int decimation = 4;
        int maxRange = 4.0;
        int minRange = 0.0;
        float noiseRadius = 0.0f;
        int noiseMinNeighbors = 5;
        bool exportImages = true;
        bool texture = true;
        cv::Mat tmpDepth;
        rtabmap::LaserScan scan;
        node.sensorData().uncompressData(
          exportImages ? &rgb : 0,
          (texture || exportImages) &&
              !node.sensorData().depthOrRightCompressed().empty()
            ? &tmpDepth
            : 0,
          &scan);
        if (scan.empty()) {
          printf("Node %d doesn't have scan data, empty cloud is created.\n",
                 iter->first);
        }
        if (decimation > 1 || minRange > 0.0f || maxRange) {
          scan = rtabmap::util3d::commonFiltering(scan, decimation, minRange,
                                                  maxRange);
        }
        if (scan.hasRGB()) {
          cloud = rtabmap::util3d::laserScanToPointCloudRGB(
            scan, scan.localTransform());
          if (noiseRadius > 0.0f && noiseMinNeighbors > 0) {
            indices = rtabmap::util3d::radiusFiltering(cloud, noiseRadius,
                                                       noiseMinNeighbors);
          }
        } else {
          cloudI = rtabmap::util3d::laserScanToPointCloudI(
            scan, scan.localTransform());
          if (noiseRadius > 0.0f && noiseMinNeighbors > 0) {
            indices = rtabmap::util3d::radiusFiltering(cloudI, noiseRadius,
                                                       noiseMinNeighbors);
          }
        }
      }

      if (cloud) {
        // RCLCPP_INFO_STREAM(get_logger(),
        //     "successfully loaded cloud: " << cloud->size());
        *accumulated_cloud += *cloud;
      }
      // if (cloudI) {
      //   RCLCPP_INFO_STREAM(get_logger(),
      //       "successfully loaded cloud intensity: " << cloudI->size());
      // }
    }
    RCLCPP_INFO_STREAM(get_logger(),
                       "Loaded " << accumulated_cloud->size() << " points");

    return true;
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string rtabmap_database_path_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTABMapDatabaseExtractor>());
  rclcpp::shutdown();
  return 0;
}
