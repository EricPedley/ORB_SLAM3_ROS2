#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <csignal>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ImuMonoRealSense : public rclcpp::Node
{
  public:
    ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(std::string(PROJECT_PATH)+ "/orb_slam3/Vocabulary/ORBvoc.txt"),
      count(0),
      count2(0)
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
      auto image_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto imu_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto image_timer_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto imu_timer_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions image_options;
      image_options.callback_group = image_callback_group;
      rclcpp::SubscriptionOptions imu_options;
      imu_options.callback_group = imu_callback_group;

      // set the sensor type based on parameter
      ORB_SLAM3::System::eSensor sensor_type;
      if (sensor_type_param == "monocular")
      {
        sensor_type = ORB_SLAM3::System::MONOCULAR;
        settings_file_path = std::string(PROJECT_PATH) + "/orb_slam3/config/Monocular/RealSense_D435i.yaml";
      } else if (sensor_type_param == "imu-monocular") {
        sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
        settings_file_path = std::string(PROJECT_PATH) + "/orb_slam3/config/Monocular-Inertial/RealSense_D435i.yaml";
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
        rclcpp::shutdown();
      }

      // open the video and imu files
      std::string imu_file_name = std::string(PROJECT_PATH) + "/videos/" + video_name.substr(0, video_name.length() - 4) + ".csv";
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
        std::string timestamp_file_name = std::string(PROJECT_PATH) + "/videos/" + video_name.substr(0, video_name.length() - 4) + "_timestamps.csv";
        video_timestamp_file.open(timestamp_file_name);

        // open video file
        input_video.open(std::string(PROJECT_PATH) + "/videos/" + video_name);
      }

      if (!input_video.isOpened() && !use_live_feed)
      {
        RCLCPP_ERROR(get_logger(), "Could not open video file for reading");
      } else {
        RCLCPP_INFO(get_logger(), "Opened video file for reading");
      }

      if (!imu_file.is_open() && !use_live_feed)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not open imu file for reading: " << imu_file_name);
      } else {
        RCLCPP_INFO(get_logger(), "Opened imu file for reading");
      }

      // setup orb slam object
      pAgent = std::make_shared<ORB_SLAM3::System>(
          vocabulary_file_path, 
          settings_file_path,
          sensor_type,
          use_pangolin,
          0);
      image_scale = pAgent->GetImageScale();

      // create subscriptions
      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&ImuMonoRealSense::image_callback, this, _1), image_options);

      imu_sub = create_subscription<sensor_msgs::msg::Imu>(
          "camera/imu", 10, std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);

      // create publishers

      // create timer
      image_timer = create_wall_timer(1s/30, std::bind(&ImuMonoRealSense::image_timer_callback, this), image_timer_callback_group);
      imu_timer = create_wall_timer(1s/200, std::bind(&ImuMonoRealSense::imu_timer_callback, this));
    }

    ~ImuMonoRealSense()
    {
      if (use_live_feed) {
        vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetTrackedMapPoints();
        save_map_to_csv(map_points);
        pAgent->Shutdown();
      }
    }

  private:

    void save_map_to_csv(vector<ORB_SLAM3::MapPoint*> map_points)
    {
      std::ofstream map_file;
      // add date and time to the map file name
      std::string time_string;
      if(use_live_feed) {
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        time_string = std::ctime(&now);
        time_string = time_string.substr(0, time_string.length() - 1) + "_";
      }
      std::string map_file_name = std::string(PROJECT_PATH) + "/maps/" + video_name.substr(0, video_name.length() - 4) + "_" + time_string + "map" + ".csv";
      map_file.open(map_file_name);
      if(map_file.is_open())
      {
        // map_file << initial_orientation->a[0] << "," << initial_orientation->a[1]
        //   << "," << initial_orientation->a[2] << "," << initial_orientation->w[0]
        //   << "," << initial_orientation->w[1] << "," << initial_orientation->w[2]
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

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

      cv::Mat frame = cv_ptr->image;
      // cv::imshow("image", frame);
      // cv::waitKey(1);

      if(image_scale != 1.f) {
        cv::resize(frame, frame, cv::Size(frame.cols*image_scale, frame.rows*image_scale));
      }

      Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp, vImuMeas);

      // if (!Tcw.matrix().isIdentity() && count == 0)
      // {
      //   RCLCPP_INFO(get_logger(), "initial orientation set");
      //   *initial_orientation = vImuMeas.back();
      //   count++;
      // }
      count++;
    }

    void imu_callback(const sensor_msgs::msg::Imu msg)
    {
      if (count > 0) {
        vImuMeas.clear();
        count = 0;
      }

      ORB_SLAM3::IMU::Point imu_meas(
          static_cast<float>(msg.linear_acceleration.x), static_cast<float>(msg.linear_acceleration.y), static_cast<float>(msg.linear_acceleration.z),
          static_cast<float>(msg.angular_velocity.x), static_cast<float>(msg.angular_velocity.y), static_cast<float>(msg.angular_velocity.z),
          msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);
      vImuMeas.push_back(imu_meas);
      // RCLCPP_INFO_STREAM(get_logger(), "imu timestamp: " << std::fixed << std::setprecision(9) << msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);


    }

    void image_timer_callback()
    {
      RCLCPP_INFO_STREAM(get_logger(), "image timer callback");
      if (!use_live_feed) {
        cv::Mat frame;
        input_video >> frame;
        auto ros_image = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame).toImageMsg();

        std::string timestamp_line;
        std::getline(video_timestamp_file, timestamp_line);
        std::istringstream ss(timestamp_line);
        std::string token;
        std::vector<float> timestamp_data;
        while(std::getline(ss, token, ',')) {
          timestamp_data.push_back(std::stof(token));
        }

        timestamp = timestamp_data[0] + timestamp_data[1] * 1e-9;
        // RCLCPP_INFO_STREAM(get_logger(), "image timestamp: " << std::fixed << std::setprecision(9) << timestamp);

        if (!frame.empty()) {
          if(image_scale != 1.f) {
            cv::resize(frame, frame, cv::Size(frame.cols*image_scale, frame.rows*image_scale));
          }

          if (sensor_type_param == "monocular") {
            Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp);
          } else if (sensor_type_param == "imu-monocular"){
            Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp, vImuMeas);
          }
          count ++;

          cv::imshow("frame", frame);
          cv::waitKey(1);
        } else {
          RCLCPP_INFO_STREAM_ONCE(get_logger(), "End of video file");
          vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetTrackedMapPoints();
          save_map_to_csv(map_points);
          pAgent->Shutdown();
          rclcpp::shutdown();
        }
      }
    }

    void imu_timer_callback()
    {
      if (!use_live_feed && sensor_type_param == "imu-monocular") {
        // if the a new frame has been processed, clear the imu data
        if (count > 0) {
          vImuMeas.clear();
          count = 0;
        }
        std::string imu_line;
        std::getline(imu_file, imu_line);
        std::istringstream ss(imu_line);
        std::string token;
        std::vector<double> imu_data;
        while(std::getline(ss, token, ',')) {
          imu_data.push_back(std::stod(token));
        }
        ORB_SLAM3::IMU::Point imu_meas(imu_data[0], imu_data[1], imu_data[2],
            imu_data[3], imu_data[4], imu_data[5], imu_data[6]);
        vImuMeas.push_back(imu_meas);


        if (count2 >= 200/30) {
          count2 = 0;
          if (!use_live_feed) {
            cv::Mat frame;
            input_video >> frame;
            auto ros_image = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame).toImageMsg();

            std::string timestamp_line;
            std::getline(video_timestamp_file, timestamp_line);
            timestamp = std::stod(timestamp_line);

            if (!frame.empty()) {
              if(image_scale != 1.f) {
                cv::resize(frame, frame, cv::Size(frame.cols*image_scale, frame.rows*image_scale));
              }

              if (sensor_type_param == "monocular") {
                Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp);
              } else if (sensor_type_param == "imu-monocular"){
                Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp, vImuMeas);
              }
              count ++;

              cv::imshow("frame", frame);
              cv::waitKey(1);
            } else {
              RCLCPP_INFO_STREAM_ONCE(get_logger(), "End of video file");
              vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetTrackedMapPoints();
              save_map_to_csv(map_points);
              pAgent->Shutdown();
              rclcpp::shutdown();
            }
          }
        }
        count2++;
      }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::TimerBase::SharedPtr image_timer;
    rclcpp::TimerBase::SharedPtr imu_timer;

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
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    std::shared_ptr<ORB_SLAM3::System> pAgent;
    std::string vocabulary_file_path;
    std::string settings_file_path;
    float image_scale;
    double timestamp;

    std::shared_ptr<ORB_SLAM3::IMU::Point> initial_orientation;
    int count;
    int count2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto n = std::make_shared<ImuMonoRealSense>();
  executor.add_node(n);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
