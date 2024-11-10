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
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
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
    declare_parameter("export_images", false);
    export_images_ = get_parameter("export_images").as_bool();
    std::string rtabmap_database = get_parameter("rtabmap_db").as_string();
    rtabmap_database_path_ =
      std::string(PROJECT_PATH) + "/maps/" + rtabmap_database;

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
  std::string generate_timestamp_string()
  {
    std::time_t now = std::time(nullptr);
    std::tm *ptm = std::localtime(&now);

    std::ostringstream oss;

    oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S");

    return oss.str();
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
    std::map<int, rtabmap::Transform> rawViewpoints;
    int imagesExported = 0;
    for (std::map<int, rtabmap::Transform>::iterator iter =
           optimizedPoses.lower_bound(1);
         iter != optimizedPoses.end(); ++iter) {
      rtabmap::Signature node = nodes.find(iter->first)->second;

      // uncompress data
      std::vector<rtabmap::CameraModel> models =
        node.sensorData().cameraModels();
      std::vector<rtabmap::StereoCameraModel> stereoModels =
        node.sensorData().stereoCameraModels();

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

      if (export_images_ && !rgb.empty()) {
        std::string dirSuffix = (depth.type() != CV_16UC1 &&
                                 depth.type() != CV_32FC1 && !depth.empty())
                                  ? "left"
                                  : "rgb";
        std::string base_name = generate_timestamp_string();
        std::string dir =
          std::string(PROJECT_PATH) + "/images/" + base_name + "_" + dirSuffix;
        std::string output_dir = dir;
        if (!UDirectory::exists(dir)) {
          UDirectory::makeDir(dir);
        }
        bool exportImagesId = true;
        std::string outputPath =
          dir + "/" +
          (exportImagesId ? uNumber2Str(iter->first)
                          : uFormat("%f", node.getStamp())) +
          ".jpg";
        cv::imwrite(outputPath, rgb);
        ++imagesExported;
        if (!depth.empty()) {
          std::string ext;
          cv::Mat depthExported = depth;
          std::string outputName;
          std::string baseName =
            outputName.empty()
              ? uSplit(UFile::getName(rtabmap_database_path_), '.').front()
              : outputName;

          if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
            ext = ".jpg";
            dir = output_dir + "/" + baseName + "_right";
          } else {
            ext = ".png";
            dir = output_dir + "/" + baseName + "_depth";
            if (depth.type() == CV_32FC1) {
              depthExported = rtabmap::util2d::cvtDepthFromFloat(depth);
            }
          }
          if (!UDirectory::exists(dir)) {
            UDirectory::makeDir(dir);
          }

          outputPath = dir + "/" +
                       (exportImagesId ? uNumber2Str(iter->first)
                                       : uFormat("%f", node.getStamp())) +
                       ext;
          cv::imwrite(outputPath, depthExported);
        }

        // save calibration per image (calibration can change over time, e.g.
        // camera has auto focus)
        for (size_t i = 0; i < models.size(); ++i) {
          rtabmap::CameraModel model = models[i];
          std::string modelName =
            (exportImagesId ? uNumber2Str(iter->first)
                            : uFormat("%f", node.getStamp()));
          if (models.size() > 1) {
            modelName += "_" + uNumber2Str((int)i);
          }
          model.setName(modelName);
          std::string dir = output_dir + "/" + base_name + "_calib";
          if (!UDirectory::exists(dir)) {
            UDirectory::makeDir(dir);
          }
          model.save(dir);
        }
        for (size_t i = 0; i < stereoModels.size(); ++i) {
          rtabmap::StereoCameraModel model = stereoModels[i];
          std::string modelName =
            (exportImagesId ? uNumber2Str(iter->first)
                            : uFormat("%f", node.getStamp()));
          if (stereoModels.size() > 1) {
            modelName += "_" + uNumber2Str((int)i);
          }
          model.setName(modelName, "left", "right");
          std::string dir = output_dir + "/" + base_name + "_calib";
          if (!UDirectory::exists(dir)) {
            UDirectory::makeDir(dir);
          }
          model.save(dir);
        }
      }

      float voxelSize = 0.0f;
      float filter_ceiling = std::numeric_limits<float>::max();
      float filter_floor = 0.0f;
      if (voxelSize > 0.0f) {
        if (cloud.get() && !cloud->empty())
          cloud = rtabmap::util3d::voxelize(cloud, indices, voxelSize);
        else if (cloudI.get() && !cloudI->empty())
          cloudI = rtabmap::util3d::voxelize(cloudI, indices, voxelSize);
      }
      if (cloud.get() && !cloud->empty())
        cloud = rtabmap::util3d::transformPointCloud(cloud, iter->second);
      else if (cloudI.get() && !cloudI->empty())
        cloudI = rtabmap::util3d::transformPointCloud(cloudI, iter->second);

      if (filter_ceiling != 0.0 || filter_floor != 0.0f) {
        if (cloud.get() && !cloud->empty()) {
          cloud = rtabmap::util3d::passThrough(
            cloud, "z",
            filter_floor != 0.0f ? filter_floor
                                 : (float)std::numeric_limits<int>::min(),
            filter_ceiling != 0.0f ? filter_ceiling
                                   : (float)std::numeric_limits<int>::max());
        }
        if (cloudI.get() && !cloudI->empty()) {
          cloudI = rtabmap::util3d::passThrough(
            cloudI, "z",
            filter_floor != 0.0f ? filter_floor
                                 : (float)std::numeric_limits<int>::min(),
            filter_ceiling != 0.0f ? filter_ceiling
                                   : (float)std::numeric_limits<int>::max());
        }
      }

      rtabmap::Transform lidarViewpoint =
        iter->second * node.sensorData().laserScanRaw().localTransform();
      rawViewpoints.insert(std::make_pair(iter->first, lidarViewpoint));

      if (cloud.get() && !cloud->empty()) {
        if (assembledCloud->empty()) {
          *assembledCloud = *cloud;
        } else {
          *assembledCloud += *cloud;
          RCLCPP_INFO_STREAM(
            get_logger(), "Assembled cloud size: " << assembledCloud->size());
        }
        rawViewpointIndices.resize(assembledCloud->size(), iter->first);
      } else if (cloudI.get() && !cloudI->empty()) {
        if (assembledCloudI->empty()) {
          *assembledCloudI = *cloudI;
        } else {
          *assembledCloudI += *cloudI;
        }
        rawViewpointIndices.resize(assembledCloudI->size(), iter->first);
      }

      if (models.empty()) {
        for (size_t i = 0; i < node.sensorData().stereoCameraModels().size();
             ++i) {
          models.push_back(node.sensorData().stereoCameraModels()[i].left());
        }
      }

      robotPoses.insert(std::make_pair(iter->first, iter->second));
      cameraStamps.insert(std::make_pair(iter->first, node.getStamp()));
      if (models.empty() && node.getWeight() == -1 && !cameraModels.empty()) {
        // For intermediate nodes, use latest models
        models = cameraModels.rbegin()->second;
      }
      if (!models.empty()) {
        if (!node.sensorData().imageCompressed().empty()) {
          cameraModels.insert(std::make_pair(iter->first, models));
        }
        if (true) {
          if (cameraPoses.empty()) {
            cameraPoses.resize(models.size());
          }
          UASSERT_MSG(models.size() == cameraPoses.size(),
                      "Not all nodes have same number of cameras to export "
                      "camera poses.");
          for (size_t i = 0; i < models.size(); ++i) {
            cameraPoses[i].insert(std::make_pair(
              iter->first, iter->second * models[i].localTransform()));
          }
        }
      }
      if (!depth.empty() &&
          (depth.type() == CV_16UC1 || depth.type() == CV_32FC1)) {
        cameraDepths.insert(std::make_pair(iter->first, depth));
      }
      if (true && !node.sensorData().laserScanCompressed().empty()) {
        scanPoses.insert(std::make_pair(
          iter->first,
          iter->second *
            node.sensorData().laserScanCompressed().localTransform()));
      }
      printf("Create and assemble the clouds... done (%fs, %d points).\n",
          timer.ticks(),
          !assembledCloud->empty() ? (int)assembledCloud->size()
          : (int)assembledCloudI->size());

      if (imagesExported > 0)
        printf("%d images exported!\n", imagesExported);

      accumulated_cloud = assembledCloud;
    }
    RCLCPP_INFO_STREAM(get_logger(),
                       "Loaded " << accumulated_cloud->size() << " points");

    return true;
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud;

  std::string rtabmap_database_path_;
  bool export_images_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTABMapDatabaseExtractor>());
  rclcpp::shutdown();
  return 0;
}
