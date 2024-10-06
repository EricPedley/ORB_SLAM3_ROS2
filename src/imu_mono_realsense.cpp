#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_srvs/srv/empty.hpp>

#include <chrono>
#include <fstream>
#include <sstream>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
//
// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class ImuMonoRealSense : public rclcpp::Node
{
public:
  ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(std::string(PROJECT_PATH) +
                           "/ORB_SLAM3/Vocabulary/ORBvoc.txt")
  {

    // declare parameters
    declare_parameter("sensor_type", "imu-monocular");
    declare_parameter("use_pangolin", true);
    declare_parameter("use_live_feed", false);
    declare_parameter("video_name", "output.mp4");

    // get parameters
    sensor_type_param = get_parameter("sensor_type").as_string();
    bool use_pangolin = get_parameter("use_pangolin").as_bool();
    use_live_feed = get_parameter("use_live_feed").as_bool();
    video_name = get_parameter("video_name").as_string();

    // define callback groups
    auto image_callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto imu_callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_callback_group;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group;

    // set the sensor type based on parameter
    ORB_SLAM3::System::eSensor sensor_type;
    if (sensor_type_param == "monocular")
    {
      sensor_type = ORB_SLAM3::System::MONOCULAR;
      settings_file_path = std::string(PROJECT_PATH) +
                           "/orb_slam3/config/Monocular/RealSense_D435i.yaml";
    }
    else if (sensor_type_param == "imu-monocular")
    {
      sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
      settings_file_path = std::string(PROJECT_PATH) +
                           "/config/Monocular-Inertial/RealSense_D435i.yaml";
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
      rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "vocabulary_file_path: " << vocabulary_file_path);

    // open the video and imu files
    std::string imu_file_name = std::string(PROJECT_PATH) + "/videos/" +
                                video_name.substr(0, video_name.length() - 4) +
                                ".csv";
    if (use_live_feed)
    {
      // dont open anything
      input_video.open(0);
    }
    else
    {
      // open the imu file
      imu_file.open(imu_file_name);

      // open the timestamp file
      std::string timestamp_file_name =
        std::string(PROJECT_PATH) + "/videos/" +
        video_name.substr(0, video_name.length() - 4) + "_timestamps.csv";
      video_timestamp_file.open(timestamp_file_name);

      // open video file
      input_video.open(std::string(PROJECT_PATH) + "/videos/" + video_name);
    }

    if (!input_video.isOpened() && !use_live_feed)
    {
      RCLCPP_ERROR(get_logger(), "Could not open video file for reading");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Opened video file for reading");
    }

    if (!imu_file.is_open() && !use_live_feed)
    {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Could not open imu file for reading: " << imu_file_name);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Opened imu file for reading");
    }

    // setup orb slam object
    pAgent = std::make_shared<ORB_SLAM3::System>(
      vocabulary_file_path, settings_file_path, sensor_type, use_pangolin, 0);
    // syncThread_ = new std::thread(&ImuMonoRealSense::sync_with_imu,
    // this);

    image_scale = pAgent->GetImageScale();

    // create subscriptions
    // create qos options
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
      std::bind(&ImuMonoRealSense::slam_service_callback, this, _1, _2));
  }

  ~ImuMonoRealSense()
  {
    pAgent->Shutdown();
    pAgent->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    if (use_live_feed)
    {
      vector<ORB_SLAM3::MapPoint *> map_points = pAgent->GetTrackedMapPoints();
      save_map_to_csv(map_points);
      pAgent->Shutdown();
    }
  }

private:
  void save_map_to_csv(vector<ORB_SLAM3::MapPoint *> map_points)
  {
    std::ofstream map_file;
    // add date and time to the map file name
    std::string time_string;
    if (use_live_feed)
    {
      std::time_t now =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      time_string = std::ctime(&now);
      time_string = time_string.substr(0, time_string.length() - 1) + "_";
    }
    std::string map_file_name = std::string(PROJECT_PATH) + "/maps/" +
                                video_name.substr(0, video_name.length() - 4) +
                                "_" + time_string + "map" + ".csv";
    map_file.open(map_file_name);
    if (map_file.is_open())
    {
      // map_file << initial_orientation->a[0] << "," <<
      // initial_orientation->a[1]
      //   << "," << initial_orientation->a[2] << "," <<
      //   initial_orientation->w[0]
      //   << "," << initial_orientation->w[1] << "," <<
      //   initial_orientation->w[2]
      //   << std::endl;
      for (auto &map_point : map_points)
      {
        Eigen::Vector3f pos = map_point->GetWorldPos();
        map_file << pos[0] << "," << pos[1] << "," << pos[2] << std::endl;
      }
      map_file.close();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Could not open map file for writing");
    }
  }

  cv::Mat get_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
      return cv_ptr->image.clone();
    }
    else
    {
      std::cerr << "Error image type" << std::endl;
      return cv_ptr->image.clone();
    }
  }

  void slam_service_callback(const std_srvs::srv::Empty::Request::SharedPtr,
                             const std_srvs::srv::Empty::Response::SharedPtr)
  {
    // RCLCPP_INFO(get_logger(), "SLAM service called");
    // pAgent->SaveTrajectoryTUM("CameraTrajectory.txt");
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // bufMutexImg_.lock();

    if (!imgBuf_.empty())
      imgBuf_.pop();
    imgBuf_.push(msg);

    // bufMutexImg_.unlock();

    // std::unique_lock<std::mutex> img_lock(bufMutexImg_, std::defer_lock);
    // std::unique_lock<std::mutex> imu_lock(bufMutex_, std::defer_lock);

    // std::lock(img_lock, imu_lock);

    bufMutexImg_.lock();
    if (!imgBuf_.empty() && !imuBuf_.empty())
    {
      auto imgPtr = imgBuf_.front();
      imgBuf_.pop(); // Safely pop the image from the buffer here
      bufMutexImg_.unlock();
      double tImage =
        imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;

      cv::Mat imageFrame = get_image(imgPtr); // Process image before popping
      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      std::stringstream imu_data_stream;

      bufMutex_.lock();
      while (!imuBuf_.empty() &&
             imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9 <=
               tImage)
      {
        auto imuPtr = imuBuf_.front();
        imuBuf_.pop();
        bufMutex_.unlock();
        double tIMU =
          imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;
        // double tIMUshort = fmod(tIMU, 100);

        cv::Point3f acc(imuPtr->linear_acceleration.x,
                        imuPtr->linear_acceleration.y,
                        imuPtr->linear_acceleration.z);
        cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y,
                        imuPtr->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));

        // Debug info
        // imu_data_stream << "IMU at " << std::fixed <<
        // std::setprecision(6)
        // << tIMUshort << " - Acc: [" << acc << "], Gyr: [" << gyr <<
        // "]\n";
      }

      // imgBuf_.pop(); // Safely pop the image from the buffer here

      if (vImuMeas.empty())
      {
        RCLCPP_WARN(this->get_logger(),
                    "No valid IMU data available for the current frame "
                    "at time %.6f.",
                    tImage);
        return; // Skip processing this frame
      }

      cv::imshow("test", imageFrame);
      cv::waitKey(1);
      try
      {
        pAgent->TrackMonocular(imageFrame, tImage, vImuMeas);
        // RCLCPP_INFO_STREAM(get_logger(), "Image at "
        //                                      << tImage
        //                                      << " processed with IMU
        //                                      "
        //                                         "data: \n"
        //                                      <<
        //                                      imu_data_stream.str());
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "SLAM processing exception: %s",
                     e.what());
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {

    if (!std::isnan(msg.linear_acceleration.x) &&
        !std::isnan(msg.linear_acceleration.y) &&
        !std::isnan(msg.linear_acceleration.z) &&
        !std::isnan(msg.angular_velocity.x) &&
        !std::isnan(msg.angular_velocity.y) &&
        !std::isnan(msg.angular_velocity.z))
    {
      bufMutex_.lock();
      const sensor_msgs::msg::Imu::SharedPtr msg_ptr =
        std::make_shared<sensor_msgs::msg::Imu>(msg);
      imuBuf_.push(msg_ptr);
      bufMutex_.unlock();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid IMU data - Rxd NaN");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr slam_service;

  sensor_msgs::msg::Imu imu_msg;

  bool use_live_feed;
  std::string video_name;
  std::string sensor_type_param;

  cv::VideoCapture input_video;
  std::ifstream imu_file;
  std::ifstream video_timestamp_file;

  std::vector<geometry_msgs::msg::Vector3> vGyro;
  std::vector<double> vGyro_times;
  std::vector<geometry_msgs::msg::Vector3> vAccel;
  std::vector<double> vAccel_times;

  queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
  queue<sensor_msgs::msg::Image::SharedPtr> imgBuf_;
  std::mutex bufMutex_, bufMutexImg_;
  // std::thread *syncThread_;

  std::shared_ptr<ORB_SLAM3::System> pAgent;
  std::string vocabulary_file_path;
  std::string settings_file_path;
  float image_scale;

  std::shared_ptr<ORB_SLAM3::IMU::Point> initial_orientation;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto n = std::make_shared<ImuMonoRealSense>();
  executor.add_node(n);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
