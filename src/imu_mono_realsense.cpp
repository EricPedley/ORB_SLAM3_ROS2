#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sstream>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class ImuMonoRealSense : public rclcpp::Node {
public:
  ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(std::string(PROJECT_PATH) +
                           "/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
      is_imu_initialized_(false), inertial_ba1_(false), inertial_ba2_(false)
  {

    // declare parameters
    declare_parameter("sensor_type", "imu-monocular");
    declare_parameter("use_pangolin", true);

    // get parameters

    sensor_type_param = get_parameter("sensor_type").as_string();
    bool use_pangolin = get_parameter("use_pangolin").as_bool();

    // define callback groups
    image_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    slam_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    octomap_server_client_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_callback_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;

    // set the sensor type based on parameter
    ORB_SLAM3::System::eSensor sensor_type;
    if (sensor_type_param == "monocular") {
      sensor_type = ORB_SLAM3::System::MONOCULAR;
      settings_file_path =
        std::string(PROJECT_PATH) + "/config/Monocular/RealSense_D435i.yaml";
    } else if (sensor_type_param == "imu-monocular") {
      sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
      settings_file_path = std::string(PROJECT_PATH) +
                           "/config/Monocular-Inertial/RealSense_D435i.yaml";
    } else {
      RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
      rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "vocabulary_file_path: " << vocabulary_file_path);

    // setup orb slam object
    orb_slam3_system_ = std::make_shared<ORB_SLAM3::System>(
      vocabulary_file_path, settings_file_path, sensor_type, use_pangolin, 0);

    // create publishers
    accumulated_pcl_cloud_msg_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("orb_point_cloud2", 10);
    frame_pcl_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("tracked_point_cloud2",
                                                      10);
    laser_scan_publisher_ =
      create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    pose_array_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 100);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // create subscriptions
    rclcpp::QoS image_qos(rclcpp::KeepLast(10));
    image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    image_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    image_sub = create_subscription<sensor_msgs::msg::Image>(
      "camera/camera/color/image_raw", image_qos,
      std::bind(&ImuMonoRealSense::image_callback, this, _1), image_options);

    rclcpp::QoS imu_qos(rclcpp::KeepLast(10));
    imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    imu_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/imu", imu_qos,
      std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);

    // create services
    slam_service = create_service<std_srvs::srv::Empty>(
      "slam_service",
      std::bind(&ImuMonoRealSense::slam_service_callback, this, _1, _2),
      rmw_qos_profile_services_default, slam_service_callback_group_);

    // create clients
    octomap_server_client_ = create_client<std_srvs::srv::Empty>(
      "octomap_server/reset", rmw_qos_profile_services_default,
      octomap_server_client_callback_group_);

    // tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create timer
    timer = create_wall_timer(
      1000ms, std::bind(&ImuMonoRealSense::timer_callback, this),
      timer_callback_group_);

    // prev_imu_->t = 0.0;
    // prev_imu_->a = {0.0, 0.0, 0.0};
    // prev_imu_->w = {0.0, 0.0, 0.0};
    initialize_variables();
  }

  ~ImuMonoRealSense()
  {
    orb_slam3_system_->SavePCDBinary(std::string(PROJECT_PATH) + "/maps/");
  }

private:
  void initialize_variables()
  {
    laser_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser_scan_->header.frame_id = "point_cloud";
    laser_scan_->scan_time = 0.0;
    laser_scan_->time_increment = 0.0;
    laser_scan_->angle_min = -M_PI;
    laser_scan_->angle_max = M_PI;
    laser_scan_->angle_increment = M_PI / 180.0;
    laser_scan_->range_min = 0.00;
    laser_scan_->range_max = 100.0;

    pose_array_ = geometry_msgs::msg::PoseArray();
    pose_array_.header.frame_id = "point_cloud";

    accumulated_pcl_cloud_msg_ = sensor_msgs::msg::PointCloud2();
    accumulated_pcl_cloud_msg_.header.frame_id = "point_cloud";
  }

  std::string generate_timestamp_string()
  {
    std::time_t now = std::time(nullptr);
    std::tm *ptm = std::localtime(&now);

    std::ostringstream oss;

    oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S") << ".mp4";

    return oss.str();
  }

  cv::Mat get_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
      return cv_ptr->image.clone();
    } else {
      std::cerr << "Error image type" << std::endl;
      return cv_ptr->image.clone();
    }
  }

  double normalize_angle(double rad)
  {
    if (rad < 0) {
      return rad + 2 * M_PI;
    } else if (rad > 2 * M_PI) {
      return rad - 2 * M_PI;
    } else {
      return rad;
    }
  }

  void
  point_cloud_to_laser_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
  {
    laser_scan->ranges.resize(
      (laser_scan_->angle_max - laser_scan_->angle_min) /
      laser_scan_->angle_increment);
    std::fill(laser_scan->ranges.begin(), laser_scan->ranges.end(), 0.0);

    for (size_t i = 0; i < cloud->points.size(); i++) {
      if (cloud->points.at(i).x <= 1e-6 && cloud->points.at(i).y <= 1e-6) {
        cloud->points.erase(cloud->points.begin() + i);
        continue;
      }
      float angle = std::atan2(cloud->points.at(i).y, cloud->points.at(i).x);
      if (angle < -M_PI || angle > M_PI) {
        continue;
      }
      angle += M_PI;
      angle = normalize_angle(angle);
      size_t index = angle / laser_scan->angle_increment;
      float distance = std::sqrt(cloud->points.at(i).x * cloud->points.at(i).x +
                                 cloud->points.at(i).y * cloud->points.at(i).y);
      if (index < 0 || index >= laser_scan->ranges.size()) {
        continue;
      }
      if (laser_scan->ranges.at(index) <= 1e-6 ||
          distance < laser_scan->ranges.at(index)) {
        laser_scan->ranges[index] = distance;
      }
    }
  }

  void slam_service_callback(const std_srvs::srv::Empty::Request::SharedPtr,
                             const std_srvs::srv::Empty::Response::SharedPtr)
  {
    orb_slam3_system_->SavePCDBinary(std::string(PROJECT_PATH) + "/maps/");
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    img_buf_.push(msg);

    // begin to empty the img_buf_ queue, which is full of other queues
    while (!img_buf_.empty()) {
      // grab the oldest image
      auto imgPtr = img_buf_.front();
      img_buf_.pop();

      cv::Mat imageFrame = get_image(imgPtr);
      double tImage =
        imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;

      vector<ORB_SLAM3::IMU::Point> vImuMeas;

      // package all the imu data for this image for orbslam3 to process
      buf_mutex_imu_.lock();
      while (!imu_buf_.empty()) {
        auto imuPtr = imu_buf_.front();
        imu_buf_.pop();
        double tIMU =
          imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9;

        cv::Point3f acc(imuPtr->linear_acceleration.x,
                        imuPtr->linear_acceleration.y,
                        imuPtr->linear_acceleration.z);
        cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y,
                        imuPtr->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));
      }

      buf_mutex_imu_.unlock();

      if (vImuMeas.empty() && sensor_type_param == "imu-monocular") {
        // RCLCPP_WARN(get_logger(),
        //             "No valid IMU data available for the current frame "
        //             "at time %.6f.",
        //             tImage);
        return;
      }

      try {
        if (sensor_type_param == "monocular") {
          orb_slam3_system_->TrackMonocular(imageFrame, tImage);
        } else {
          if (vImuMeas.size() > 1) {
            auto Tcw =
              orb_slam3_system_->TrackMonocular(imageFrame, tImage, vImuMeas);
            if (orb_slam3_system_->isImuInitialized() ||
                orb_slam3_system_->GetInertialBA1() ||
                orb_slam3_system_->GetInertialBA2()) {
              auto Tcb = orb_slam3_system_->GetCurrentPoseImu();
              // auto Tco = Two * Tcw.inverse();
              auto Tco = Tcw.inverse() * Tcb;
              if (pose_array_.poses.size() == 0) {
                prev_position_ = Tco.translation();
                prev_time_ = tImage;
                prev_imu_ =
                  std::make_shared<ORB_SLAM3::IMU::Point>(vImuMeas.back());
              } else {
                // // calculate linear velocity
                // Eigen::Vector3f velocity =
                //   (Tco.translation() - prev_position_) / (tImage -
                //   prev_time_);
                // prev_position_ = Tco.translation();
                // prev_time_ = tImage;
                // prev_imu_ =
                //   std::make_shared<ORB_SLAM3::IMU::Point>(vImuMeas.back());
                //
                // // calculate angular velocity
                // Eigen::Vector3f angular_velocity =
                //   (vImuMeas.back().w - prev_imu_->w) / (tImage -
                //   prev_imu_->t);
                //
                // nav_msgs::msg::Odometry odom;
                // odom_msg_.header.stamp = get_clock()->now();
                // odom_msg_.header.frame_id = "map";
                // odom_msg_.child_frame_id = "odom";
                // odom_msg_.pose.pose.position.x = Tco.translation().x();
                // odom_msg_.pose.pose.position.y = Tco.translation().y();
                // odom_msg_.pose.pose.position.z = Tco.translation().z();
                // odom_msg_.pose.pose.orientation.x =
                // Tco.unit_quaternion().x(); odom_msg_.pose.pose.orientation.y
                // = Tco.unit_quaternion().y();
                // odom_msg_.pose.pose.orientation.z =
                // Tco.unit_quaternion().z(); odom_msg_.pose.pose.orientation.w
                // = Tco.unit_quaternion().w(); odom_msg_.twist.twist.linear.x =
                // velocity.x(); odom_msg_.twist.twist.linear.y = velocity.y();
                // odom_msg_.twist.twist.linear.z = velocity.z();
                // odom_msg_.twist.twist.angular.x = angular_velocity.x();
                // odom_msg_.twist.twist.angular.y = angular_velocity.y();
                // odom_msg_.twist.twist.angular.z = angular_velocity.z();
                // odom_publisher_->publish(odom_msg_);
              }
              geometry_msgs::msg::Pose pose;
              pose.position.x = Tco.translation().x();
              pose.position.y = Tco.translation().y();
              // pose.position.z = Tco.translation().z();
              pose.orientation.x = Tco.unit_quaternion().x();
              pose.orientation.y = Tco.unit_quaternion().y();
              pose.orientation.z = Tco.unit_quaternion().z();
              pose.orientation.w = Tco.unit_quaternion().w();
              pose_array_.header.stamp = get_clock()->now();
              pose_array_.poses.push_back(pose);

              geometry_msgs::msg::TransformStamped Tco_tf;
              Tco_tf.header.stamp = get_clock()->now();
              Tco_tf.header.frame_id = "map";
              Tco_tf.child_frame_id = "odom";
              Tco_tf.transform.translation.x = Tco.translation().x();
              Tco_tf.transform.translation.y = Tco.translation().y();
              // Tco_tf.transform.translation.z = Tco.translation().z();
              Tco_tf.transform.rotation.x = Tco.unit_quaternion().x();
              Tco_tf.transform.rotation.y = Tco.unit_quaternion().y();
              Tco_tf.transform.rotation.z = Tco.unit_quaternion().z();
              Tco_tf.transform.rotation.w = Tco.unit_quaternion().w();
              tf_broadcaster->sendTransform(Tco_tf);

              geometry_msgs::msg::TransformStamped base_link_tf;
              base_link_tf.header.stamp = get_clock()->now();
              base_link_tf.header.frame_id = "odom";
              base_link_tf.child_frame_id = "base_link";
              tf_broadcaster->sendTransform(base_link_tf);

              geometry_msgs::msg::TransformStamped t;
              t.header.stamp = get_clock()->now();
              t.header.frame_id = "map";
              t.child_frame_id = "point_cloud";

              t.transform.translation.z = 2;
              tf_broadcaster->sendTransform(t);

              // pcl::PointCloud<pcl::PointXYZ> new_pcl_cloud =
              //   orb_slam3_system_->GetTrackedMapPointsPCL();

              frame_pcl_cloud_ = orb_slam3_system_->GetTrackedMapPointsPCL();

              // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr =
              //   std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(new_pcl_cloud);
              //
              pcl::PointCloud<pcl::PointXYZ>::Ptr frame_ptr =
                std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
                  frame_pcl_cloud_);

              point_cloud_to_laser_scan(frame_ptr, laser_scan_);

              // voxel grid filter
              pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
              pcl::VoxelGrid<pcl::PointXYZ> vg;
              vg.setInputCloud(frame_ptr);
              vg.setLeafSize(0.05f, 0.05f, 0.05f);
              vg.filter(*voxel_cloud);

              // statistical outlier removal

              pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
              sor.setInputCloud(voxel_cloud);
              sor.setMeanK(100);
              sor.setStddevMulThresh(0.1);
              sor.filter(*sor_cloud);

              pcl::toROSMsg(*sor_cloud, frame_pcl_cloud_msg);

              // pcl_cloud_ = orb_slam3_system_->GetMapPCL();
              accumulated_pcl_cloud_ += frame_pcl_cloud_;
              pcl::toROSMsg(accumulated_pcl_cloud_, accumulated_pcl_cloud_msg_);

              accumulated_pcl_cloud_msg_.header.frame_id = "point_cloud";
              accumulated_pcl_cloud_msg_.header.stamp = get_clock()->now();

              frame_pcl_cloud_msg.header.frame_id = "point_cloud";
              frame_pcl_cloud_msg.header.stamp = get_clock()->now();

              laser_scan_->header.stamp = get_clock()->now();
            }
          }
        }

        // if (!inertial_ba1_ && orb_slam3_system_->GetInertialBA1()) {
        //   inertial_ba1_ = true;
        //   accumulated_pcl_cloud_ = orb_slam3_system_->GetMapPCL();
        //   frame_pcl_cloud_.clear();
        //   pose_array_.poses.clear();
        //   pose_array_.header.stamp = get_clock()->now();
        //   for (const auto &pose : orb_slam3_system_->GetTcoPoses()) {
        //     geometry_msgs::msg::Pose pose_msg;
        //     pose_msg.position.x = pose.translation().x();
        //     pose_msg.position.y = pose.translation().y();
        //     pose_msg.position.z = pose.translation().z();
        //     pose_msg.orientation.x = pose.unit_quaternion().x();
        //     pose_msg.orientation.y = pose.unit_quaternion().y();
        //     pose_msg.orientation.z = pose.unit_quaternion().z();
        //     pose_msg.orientation.w = pose.unit_quaternion().w();
        //     pose_array_.poses.push_back(pose_msg);
        //   }
        //   RCLCPP_INFO(get_logger(), "Inertial BA1 complete");
        // }
        //
        // if (!inertial_ba2_ && orb_slam3_system_->GetInertialBA2()) {
        //   inertial_ba2_ = true;
        //   accumulated_pcl_cloud_ = orb_slam3_system_->GetMapPCL();
        //   frame_pcl_cloud_.clear();
        //   pose_array_.poses.clear();
        //   pose_array_.header.stamp = get_clock()->now();
        //   for (const auto &pose : orb_slam3_system_->GetTcoPoses()) {
        //     geometry_msgs::msg::Pose pose_msg;
        //     pose_msg.position.x = pose.translation().x();
        //     pose_msg.position.y = pose.translation().y();
        //     pose_msg.position.z = pose.translation().z();
        //     pose_msg.orientation.x = pose.unit_quaternion().x();
        //     pose_msg.orientation.y = pose.unit_quaternion().y();
        //     pose_msg.orientation.z = pose.unit_quaternion().z();
        //     pose_msg.orientation.w = pose.unit_quaternion().w();
        //     pose_array_.poses.push_back(pose_msg);
        //   }
        //   RCLCPP_INFO(get_logger(), "Inertial BA2 complete");
        // }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "SLAM processing exception: %s", e.what());
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    buf_mutex_imu_.lock();
    if (!std::isnan(msg.linear_acceleration.x) &&
        !std::isnan(msg.linear_acceleration.y) &&
        !std::isnan(msg.linear_acceleration.z) &&
        !std::isnan(msg.angular_velocity.x) &&
        !std::isnan(msg.angular_velocity.y) &&
        !std::isnan(msg.angular_velocity.z)) {
      const sensor_msgs::msg::Imu::SharedPtr msg_ptr =
        std::make_shared<sensor_msgs::msg::Imu>(msg);
      imu_buf_.push(msg_ptr);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid IMU data - nan");
    }
    buf_mutex_imu_.unlock();
  }

  void timer_callback()
  {
    unique_lock<mutex> lock(orbslam3_mutex_);

    geometry_msgs::msg::Pose pose;
    if (orb_slam3_system_->isImuInitialized() ||
        orb_slam3_system_->GetInertialBA1() ||
        orb_slam3_system_->GetInertialBA2()) {
      if (pose_array_.poses.size() > 1000) {
        pose_array_.poses.erase(pose_array_.poses.begin());
      }

      // publish the variables I've been accumulating in the image callback
      pose_array_publisher_->publish(pose_array_);
      // odom_publisher_->publish(odom_msg_);
      frame_pcl_cloud_publisher_->publish(frame_pcl_cloud_msg);
      accumulated_pcl_cloud_msg_publisher_->publish(accumulated_pcl_cloud_msg_);
      laser_scan_publisher_->publish(*laser_scan_);
    } else {
      octomap_server_client_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>());
      accumulated_pcl_cloud_ = orb_slam3_system_->GetMapPCL();
      frame_pcl_cloud_.clear();
      initialize_variables();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    accumulated_pcl_cloud_msg_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    frame_pcl_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
    laser_scan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    pose_array_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr slam_service;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_server_client_;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr slam_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr octomap_server_client_callback_group_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  sensor_msgs::msg::Imu imu_msg;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_scan_;

  nav_msgs::msg::Odometry odom_msg_;

  std::string sensor_type_param;

  std::vector<geometry_msgs::msg::Vector3> vGyro;
  std::vector<double> vGyro_times;
  std::vector<geometry_msgs::msg::Vector3> vAccel;
  std::vector<double> vAccel_times;

  queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
  queue<sensor_msgs::msg::Image::SharedPtr> img_buf_;
  std::mutex buf_mutex_imu_, buf_mutex_img_, orbslam3_mutex_;

  std::shared_ptr<ORB_SLAM3::System> orb_slam3_system_;
  std::string vocabulary_file_path;
  std::string settings_file_path;

  sensor_msgs::msg::PointCloud2 accumulated_pcl_cloud_msg_;
  sensor_msgs::msg::PointCloud2 frame_pcl_cloud_msg;
  pcl::PointCloud<pcl::PointXYZ> frame_pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZ> accumulated_pcl_cloud_;

  geometry_msgs::msg::PoseArray pose_array_;

  std::shared_ptr<ORB_SLAM3::IMU::Point> initial_orientation;
  Eigen::Vector3f prev_position_;
  Eigen::Quaternionf prev_orientation_;
  std::shared_ptr<ORB_SLAM3::IMU::Point> prev_imu_;
  double prev_time_;

  bool is_imu_initialized_;
  bool inertial_ba1_;
  bool inertial_ba2_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuMonoRealSense>());
  rclcpp::shutdown();
  return 0;
}
