#include <chrono>
#include <functional>
#include <memory>
#include <nav2_map_server/map_io.hpp>
#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <rtabmap/core/ProgressState.h>
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
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
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

    rtabmap_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    model_path_ = std::string(PROJECT_PATH) + "/models/" + "yolox_s.onnx";
    RCLCPP_INFO_STREAM(get_logger(), "Loading model: " << model_path_);
    net_ = cv::dnn::readNet(model_path_);

    if (!load_rtabmap_db(rtabmap_database_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load database");
      rclcpp::shutdown();
    }

    // create publishers
    point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "rtabmap_point_cloud", 10);
    occupancy_grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "rtabmap_occupancy_grid", 10);
    image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("rtabmap_image", 10);

    // create timer
    timer_ = create_wall_timer(
      500ms, std::bind(&RTABMapDatabaseExtractor::timer_callback, this));

    rclcpp::on_shutdown([this]() {
      pcl::io::savePCDFileBinary(std::string(PROJECT_PATH) + "/maps/" +
                                   generate_timestamp_string() + ".pcd",
                                 *rtabmap_cloud_);
      nav2_map_server::SaveParameters save_params;
      save_params.map_file_name = std::string(PROJECT_PATH) +
                                  "/occupancy_grids/" +
                                  generate_timestamp_string();
      save_params.image_format = "pgm";
      save_params.free_thresh = 0.196;
      save_params.occupied_thresh = 0.65;
      nav2_map_server::saveMapToFile(*rtabmap_occupancy_grid_, save_params);
    });
  }

private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  filter_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    // statistical outlier removal
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100); // increase for more permissive, decrease for less
    sor.setStddevMulThresh(
      1.0); // increase for more permissive, decrease for less
    sor.filter(*sor_cloud);

    // radius outlier removal
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier;
    radius_outlier.setInputCloud(sor_cloud);
    radius_outlier.setRadiusSearch(
      0.5); // Adjust based on spacing in the point cloud
    radius_outlier.setMinNeighborsInRadius(
      3); // Increase for more aggressive outlier removal
    radius_outlier.filter(*radius_cloud);
    radius_cloud->width = radius_cloud->points.size();

    return radius_cloud;
  }
  nav_msgs::msg::OccupancyGrid::SharedPtr
  point_cloud_to_occupancy_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    // calculate the centroid
    Eigen::Matrix<float, 4, 1> centroid;
    pcl::ConstCloudIterator<pcl::PointXYZRGB> cloud_iterator(*cloud);
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
    occupancy_grid->header.frame_id = "live_map";
    occupancy_grid->header.stamp = get_clock()->now();
    occupancy_grid->info.resolution = 0.05;
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
  void timer_callback()
  {
    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl::toPCLPointCloud2(*rtabmap_cloud_, pcl_pc2);
    // sensor_msgs::msg::PointCloud2 msg;
    // pcl_conversions::fromPCL(pcl_pc2, msg);
    // msg.header.frame_id = "map";
    // msg.header.stamp = get_clock()->now();
    // point_cloud_publisher_->publish(msg);

    rtabmap_occupancy_grid_->header.stamp = get_clock()->now();
    occupancy_grid_publisher_->publish(*rtabmap_occupancy_grid_);

    if (!images_.empty()) {
      RCLCPP_INFO_STREAM(get_logger(), "images size: " << images_.size());
      cv_bridge::CvImage cv_image;
      cv_image.header.stamp = get_clock()->now();
      cv_image.encoding = "bgr8";
      cv_image.image = images_.front();
      images_.erase(images_.begin());
      image_publisher_->publish(*cv_image.toImageMsg());
    } else {
      RCLCPP_INFO(get_logger(), "No more images to publish");
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*rtabmap_cloud_, pcl_pc2);
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
    RCLCPP_INFO(get_logger(), "Optimizing the map... done (%fs, poses=%d).\n",
                timer.ticks(), (int)optimizedPoses.size());

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
          RCLCPP_INFO(
            get_logger(),
            "Node %d doesn't have scan data, empty cloud is created.\n",
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

      node.sensorData().uncompressData(&rgb, &depth);
      if (!rgb.empty()) {
        images_.push_back(rgb);
      }

      if (!depth.empty()) {
        RCLCPP_INFO_STREAM(get_logger(),
                           "depth width: " << depth.cols
                                           << " height: " << depth.rows);
        // cv::imshow("Depth", depth);
        // cv::waitKey(50);
      }

      if(!rgb.empty()) {
        int center_x = rgb.cols / 2;
        int center_y = rgb.rows / 2;

        int crop_width = 640;
        int crop_height = 640;

        cv::Mat crop =
          rgb(cv::Rect(center_x - crop_width / 2, center_y - crop_height / 2,
                       crop_width, crop_height));

        cv::Mat resized_image;
        cv::resize(crop, resized_image, cv::Size(640, 640));

        RCLCPP_INFO_STREAM(get_logger(), "resized_image width: "
                                           << resized_image.cols
                                           << " height: " <<
                                           resized_image.rows);
        RCLCPP_INFO_STREAM(get_logger(), "resized_image channels: "
                                           << resized_image.channels());
        RCLCPP_INFO_STREAM(get_logger(), "resized_image type: "
                                           << resized_image.type());

        cv::Mat blob =
          cv::dnn::blobFromImage(resized_image, 1 / 255.0, cv::Size(640,
          640),
                                 cv::Scalar(0, 0, 0), true, false);
        if(blob.empty()) {
          RCLCPP_ERROR(get_logger(), "Failed to create blob");
          return false;
        }
        net_.setInput(blob);
        std::vector<cv::Mat> detections;
        net_.forward(detections);

        RCLCPP_INFO_STREAM(get_logger(),
                           "detections size: " << detections.size());

        float confidence_threshold = 0.5;
        for (size_t i = 0; i < detections.size(); ++i) {
          float *data = (float *)detections[i].data;

          // Loop through each detection
          for (int j = 0; j < detections[i].rows;
               ++j, data += detections[i].cols) {
            float confidence = data[4]; // Confidence score of the detection
            RCLCPP_INFO_STREAM(get_logger(), "confidence: " << confidence);
            RCLCPP_INFO_STREAM(get_logger(),
                               "confidence_threshold: " <<
                               confidence_threshold);
            if (confidence > confidence_threshold) {
              int left = (int)(data[0] * resized_image.cols);   // x1
              int top = (int)(data[1] * resized_image.rows);    // y1
              int right = (int)(data[2] * resized_image.cols);  // x2
              int bottom = (int)(data[3] * resized_image.rows); // y2

              // Draw bounding box
              cv::rectangle(resized_image, cv::Point(left, top),
                            cv::Point(right, bottom), cv::Scalar(0, 255, 0),
                            2);

              // Display confidence text
              std::string label = cv::format("%.2f", confidence);
              cv::putText(resized_image, label, cv::Point(left, top - 10),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5,
                          cv::Scalar(255, 255, 255), 2);
            }
          }
        }

        // Display the output image with bounding boxes
        cv::imshow("Detection", resized_image);
        cv::waitKey(50);
      }

      // saving images stuff
      // if (!rgb.empty()) {
      //   std::string dirSuffix = (depth.type() != CV_16UC1 &&
      //                            depth.type() != CV_32FC1 && !depth.empty())
      //                             ? "left"
      //                             : "rgb";
      //   std::string base_name = generate_timestamp_string();
      //   std::string dir =
      //     std::string(PROJECT_PATH) + "/images/" + base_name + "_" +
      //     dirSuffix;
      //   std::string output_dir = dir;
      //   if (!UDirectory::exists(dir)) {
      //     UDirectory::makeDir(dir);
      //   }
      // bool exportImagesId = true;
      // std::string outputPath =
      //   dir + "/" +
      //   (exportImagesId ? uNumber2Str(iter->first)
      //                   : uFormat("%f", node.getStamp())) +
      //   ".jpg";
      // cv::imwrite(outputPath, rgb);
      // ++imagesExported;
      // if (!depth.empty()) {
      //   std::string ext;
      //   cv::Mat depthExported = depth;
      //   std::string outputName;
      //   std::string baseName =
      //     outputName.empty()
      //       ? uSplit(UFile::getName(rtabmap_database_path_), '.').front()
      //       : outputName;
      //
      //   if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
      //     ext = ".jpg";
      //     dir = output_dir + "/" + baseName + "_right";
      //   } else {
      //     ext = ".png";
      //     dir = output_dir + "/" + baseName + "_depth";
      //     if (depth.type() == CV_32FC1) {
      //       depthExported = rtabmap::util2d::cvtDepthFromFloat(depth);
      //     }
      //   }
      //   if (!UDirectory::exists(dir)) {
      //     UDirectory::makeDir(dir);
      //   }
      //
      //   outputPath = dir + "/" +
      //                (exportImagesId ? uNumber2Str(iter->first)
      //                                : uFormat("%f", node.getStamp())) +
      //                ext;
      //   cv::imwrite(outputPath, depthExported);
      // }

      // save calibration per image (calibration can change over time, e.g.
      // camera has auto focus)
      // for (size_t i = 0; i < models.size(); ++i) {
      //   rtabmap::CameraModel model = models[i];
      //   std::string modelName =
      //     (exportImagesId ? uNumber2Str(iter->first)
      //                     : uFormat("%f", node.getStamp()));
      //   if (models.size() > 1) {
      //     modelName += "_" + uNumber2Str((int)i);
      //   }
      //   model.setName(modelName);
      //   std::string dir = output_dir + "/" + base_name + "_calib";
      //   if (!UDirectory::exists(dir)) {
      //     UDirectory::makeDir(dir);
      //   }
      //   model.save(dir);
      // }
      // for (size_t i = 0; i < stereoModels.size(); ++i) {
      //   rtabmap::StereoCameraModel model = stereoModels[i];
      //   std::string modelName =
      //     (exportImagesId ? uNumber2Str(iter->first)
      //                     : uFormat("%f", node.getStamp()));
      //   if (stereoModels.size() > 1) {
      //     modelName += "_" + uNumber2Str((int)i);
      //   }
      //   model.setName(modelName, "left", "right");
      //   std::string dir = output_dir + "/" + base_name + "_calib";
      //   if (!UDirectory::exists(dir)) {
      //     UDirectory::makeDir(dir);
      //   }
      //   // model.save(dir);
      // }
      // }

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
          clouds_.push_back(cloud);
        } else {
          *assembledCloud += *cloud;
          clouds_.push_back(cloud);
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
      RCLCPP_INFO(get_logger(),
                  "Create and assemble the clouds... done (%fs, %d points).\n",
                  timer.ticks(),
                  !assembledCloud->empty() ? (int)assembledCloud->size()
                                           : (int)assembledCloudI->size());

      if (imagesExported > 0)
        RCLCPP_INFO(get_logger(), "%d images exported!\n", imagesExported);

      rtabmap_cloud_ = assembledCloud;
    }

    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*rtabmap_cloud_, *cloud2);

    for (std::map<int, std::vector<rtabmap::CameraModel>>::iterator iter =
           cameraModels.begin();
         iter != cameraModels.end(); ++iter) {
      cv::Mat frame =
        cv::Mat::zeros(iter->second.front().imageHeight(),
                       iter->second.front().imageWidth(), CV_8UC3);
      cv::Mat depth(iter->second.front().imageHeight(),
                    iter->second.front().imageWidth() * iter->second.size(),
                    CV_32FC1);
      for (size_t i = 0; i < iter->second.size(); ++i) {
        cv::Mat subDepth = rtabmap::util3d::projectCloudToCamera(
          iter->second.at(i).imageSize(), iter->second.at(i).K(), cloud2,
          robotPoses.at(iter->first) * iter->second.at(i).localTransform());
        subDepth.copyTo(
          depth(cv::Range::all(),
                cv::Range(i * iter->second.front().imageWidth(),
                          (i + 1) * iter->second.front().imageWidth())));
      }

      for (int y = 0; y < depth.rows; ++y) {
        for (int x = 0; x < depth.cols; ++x) {
          if (depth.at<float>(y, x) > 0.0f) // Valid depth
          {
            cv::circle(frame, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
          }
        }
      }

      cv::imshow("Overlay", frame);
      cv::waitKey(50); // Press any key to continue

      depth = rtabmap::util2d::cvtDepthFromFloat(depth);
      // cv::imshow("depth", depth);
      // cv::waitKey(50);
      // std::string
      // outputPath=dir+"/"+(exportImagesId?uNumber2Str(iter->first):uFormat("%f",cameraStamps.at(iter->first)))+".png";
      // cv::imwrite(outputPath, depth);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithoutNormals(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr rawAssembledCloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*rtabmap_cloud_, *cloudWithoutNormals);
    rawAssembledCloud = cloudWithoutNormals;

    pcl::PointCloud<pcl::Normal>::Ptr normals =
      rtabmap::util3d::computeNormals(cloudWithoutNormals, 20, 0);

    bool groundNormalsUp = true;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudToExport(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIToExport(
      new pcl::PointCloud<pcl::PointXYZINormal>);
    if (!assembledCloud->empty()) {
      UASSERT(assembledCloud->size() == normals->size());
      pcl::concatenateFields(*assembledCloud, *normals, *cloudToExport);
      RCLCPP_INFO(
        get_logger(),
        "Computing normals of the assembled cloud... done! (%fs, %d points)\n",
        timer.ticks(), (int)assembledCloud->size());
      assembledCloud->clear();

      // adjust with point of views
      RCLCPP_INFO(
        get_logger(),
        "Adjust normals to viewpoints of the assembled cloud... (%d points)\n",
        (int)cloudToExport->size());
      rtabmap::util3d::adjustNormalsToViewPoints(
        rawViewpoints, rawAssembledCloud, rawViewpointIndices, cloudToExport,
        groundNormalsUp);
      RCLCPP_INFO(
        get_logger(),
        "Adjust normals to viewpoints of the assembled cloud... (%fs, %d "
        "points)\n",
        timer.ticks(), (int)cloudToExport->size());
    } else if (!assembledCloudI->empty()) {
      UASSERT(assembledCloudI->size() == normals->size());
      pcl::concatenateFields(*assembledCloudI, *normals, *cloudIToExport);
      RCLCPP_INFO(
        get_logger(),
        "Computing normals of the assembled cloud... done! (%fs, %d points)\n",
        timer.ticks(), (int)assembledCloudI->size());
      assembledCloudI->clear();

      // adjust with point of views
      RCLCPP_INFO(
        get_logger(),
        "Adjust normals to viewpoints of the assembled cloud... (%d points)\n",
        (int)cloudIToExport->size());
      rtabmap::util3d::adjustNormalsToViewPoints(
        rawViewpoints, rawAssembledCloud, rawViewpointIndices, cloudIToExport,
        groundNormalsUp);
      RCLCPP_INFO(
        get_logger(),
        "Adjust normals to viewpoints of the assembled cloud... (%fs, %d "
        "points)\n",
        timer.ticks(), (int)cloudIToExport->size());
    }

    std::vector<std::pair<std::pair<int, int>, pcl::PointXY>> pointToPixel;
    float textureRange = 0.0f;
    float textureAngle = 0.0f;
    std::vector<float> textureRoiRatios;
    cv::Mat projMask;
    bool distanceToCamPolicy = false;
    const rtabmap::ProgressState progressState;
    pointToPixel = rtabmap::util3d::projectCloudToCameras(
      *cloudToExport, robotPoses, cameraModels, textureRange, textureAngle,
      textureRoiRatios, projMask, distanceToCamPolicy, &progressState);

    // color the cloud
    std::vector<int> pointToCamId;
    std::vector<float> pointToCamIntensity;
    pointToCamId.resize(!cloudToExport->empty() ? cloudToExport->size()
                                                : cloudIToExport->size());

    UASSERT(pointToPixel.empty() || pointToPixel.size() == pointToCamId.size());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr assembledCloudValidPoints(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    assembledCloudValidPoints->resize(pointToCamId.size());

    int imagesDone = 1;
    for (std::map<int, rtabmap::Transform>::iterator iter = robotPoses.begin();
         iter != robotPoses.end(); ++iter) {
      int nodeID = iter->first;
      cv::Mat image;
      if (uContains(nodes, nodeID) &&
          !nodes.at(nodeID).sensorData().imageCompressed().empty()) {
        nodes.at(nodeID).sensorData().uncompressDataConst(&image, 0);
      }
      if (!image.empty()) {
        UASSERT(cameraModels.find(nodeID) != cameraModels.end());
        int modelsSize = cameraModels.at(nodeID).size();
        for (size_t i = 0; i < pointToPixel.size(); ++i) {
          int cameraIndex = pointToPixel[i].first.second;
          if (nodeID == pointToPixel[i].first.first && cameraIndex >= 0) {
            pcl::PointXYZRGBNormal pt;
            float intensity = 0;
            if (!cloudToExport->empty()) {
              pt = cloudToExport->at(i);
            } else if (!cloudIToExport->empty()) {
              pt.x = cloudIToExport->at(i).x;
              pt.y = cloudIToExport->at(i).y;
              pt.z = cloudIToExport->at(i).z;
              pt.normal_x = cloudIToExport->at(i).normal_x;
              pt.normal_y = cloudIToExport->at(i).normal_y;
              pt.normal_z = cloudIToExport->at(i).normal_z;
              intensity = cloudIToExport->at(i).intensity;
            }

            int subImageWidth = image.cols / modelsSize;
            cv::Mat subImage = image(
              cv::Range::all(), cv::Range(cameraIndex * subImageWidth,
                                          (cameraIndex + 1) * subImageWidth));

            int x = pointToPixel[i].second.x * (float)subImage.cols;
            int y = pointToPixel[i].second.y * (float)subImage.rows;
            UASSERT(x >= 0 && x < subImage.cols);
            UASSERT(y >= 0 && y < subImage.rows);

            if (subImage.type() == CV_8UC3) {
              cv::Vec3b bgr = subImage.at<cv::Vec3b>(y, x);
              pt.b = bgr[0];
              pt.g = bgr[1];
              pt.r = bgr[2];
            } else {
              UASSERT(subImage.type() == CV_8UC1);
              pt.r = pt.g = pt.b = subImage.at<unsigned char>(
                pointToPixel[i].second.y * subImage.rows,
                pointToPixel[i].second.x * subImage.cols);
            }

            int exportedId = nodeID;
            pointToCamId[i] = exportedId;
            if (!pointToCamIntensity.empty()) {
              pointToCamIntensity[i] = intensity;
            }
            assembledCloudValidPoints->at(i) = pt;
          }
        }
      }
      RCLCPP_INFO(get_logger(), "Processed %d/%d images", imagesDone++,
                  (int)robotPoses.size());
    }

    pcl::IndicesPtr validIndices(new std::vector<int>(pointToPixel.size()));
    size_t oi = 0;
    for (size_t i = 0; i < pointToPixel.size(); ++i) {
      if (pointToPixel[i].first.first <= 0) {
        pcl::PointXYZRGBNormal pt;
        float intensity = 0;
        if (!cloudToExport->empty()) {
          pt = cloudToExport->at(i);
        } else if (!cloudIToExport->empty()) {
          pt.x = cloudIToExport->at(i).x;
          pt.y = cloudIToExport->at(i).y;
          pt.z = cloudIToExport->at(i).z;
          pt.normal_x = cloudIToExport->at(i).normal_x;
          pt.normal_y = cloudIToExport->at(i).normal_y;
          pt.normal_z = cloudIToExport->at(i).normal_z;
          intensity = cloudIToExport->at(i).intensity;
        }

        pointToCamId[i] = 0; // invalid
        pt.b = 0;
        pt.g = 0;
        pt.r = 255;
        if (!pointToCamIntensity.empty()) {
          pointToCamIntensity[i] = intensity;
        }
        assembledCloudValidPoints->at(i) = pt; // red
        validIndices->at(oi++) = i;
      } else {
        validIndices->at(oi++) = i;
      }
    }

    if (oi != validIndices->size()) {
      validIndices->resize(oi);
      assembledCloudValidPoints = rtabmap::util3d::extractIndices(
        assembledCloudValidPoints, validIndices, false, false);
      std::vector<int> pointToCamIdTmp(validIndices->size());
      std::vector<float> pointToCamIntensityTmp(validIndices->size());
      for (size_t i = 0; i < validIndices->size(); ++i) {
        pointToCamIdTmp[i] = pointToCamId[validIndices->at(i)];
        pointToCamIntensityTmp[i] = pointToCamIntensity[validIndices->at(i)];
      }
      pointToCamId = pointToCamIdTmp;
      pointToCamIntensity = pointToCamIntensityTmp;
      pointToCamIdTmp.clear();
      pointToCamIntensityTmp.clear();
    }

    cloudToExport = assembledCloudValidPoints;
    cloudIToExport->clear();

    pcl::copyPointCloud(*cloudToExport, *rtabmap_cloud_);

    RCLCPP_INFO(get_logger(), "Camera projection... done! (%fs)\n",
                timer.ticks());

    rtabmap_cloud_ = filter_point_cloud(rtabmap_cloud_);
    rtabmap_occupancy_grid_ = point_cloud_to_occupancy_grid(rtabmap_cloud_);
    rtabmap_occupancy_grid_->header.frame_id = "map";

    return true;
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    point_cloud_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    occupancy_grid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rtabmap_cloud_;
  nav_msgs::msg::OccupancyGrid::SharedPtr rtabmap_occupancy_grid_;

  std::string model_path_;
  cv::dnn::Net net_;

  std::string rtabmap_database_path_;
  bool export_images_;
  std::vector<cv::Mat> images_;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTABMapDatabaseExtractor>());
  rclcpp::shutdown();
  return 0;
}
