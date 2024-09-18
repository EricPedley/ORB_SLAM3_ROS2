#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

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

#include <condition_variable>

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

class MonoRealSense : public rclcpp::Node
{
  public:
    MonoRealSense()
    : Node("monorealsense"),
      vocabulary_file_path(std::string(PROJECT_PATH)+ "/orb_slam3/Vocabulary/ORBvoc.txt.bin"),
      settings_file_path(std::string(PROJECT_PATH) + "/orb_slam3/config/Monocular/RealSense_D435i.yaml"),
      timestamp(0.0),
      count(0)
    {
      // add a parameter to the RealSense .yaml file that tells it where to save the map .aso file
      std::ofstream settings_file(settings_file_path, std::ios::app);

      // declare parameters
      declare_parameter("sensor_type", "monocular");
      declare_parameter("use_pangolin", true);
      declare_parameter("use_live_feed", false);
      declare_parameter("video_name", "output.mp4");

      // get parameters
      std::string sensor_type_param = get_parameter("sensor_type").as_string();
      bool use_pangolin = get_parameter("use_pangolin").as_bool();
      use_live_feed = get_parameter("use_live_feed").as_bool();
      video_name = get_parameter("video_name").as_string();

      // set the sensor type based on parameter
      ORB_SLAM3::System::eSensor sensor_type;
      if (sensor_type_param == "monocular")
      {
        sensor_type = ORB_SLAM3::System::MONOCULAR;
      } else if (sensor_type_param == "imu-monocular") {
        sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
        rclcpp::shutdown();
      }

      if (use_live_feed)
      {
        // dont open anything
        input_video.open(0);
      }
      else
      {
        // open the imu file
        std::string imu_file_name = std::string(PROJECT_PATH) + "/videos/" + video_name.substr(0, video_name.length() - 4) + ".csv";
        imu_file.open(imu_file_name);

        // open video file
        input_video.open(std::string(PROJECT_PATH) + "/videos/" + video_name);
      }

      if (!input_video.isOpened())
      {
        RCLCPP_ERROR(get_logger(), "Could not open video file for reading");
        rclcpp::shutdown();
      }

      if (!imu_file.is_open())
      {
        RCLCPP_ERROR(get_logger(), "Could not open imu file for reading");
        rclcpp::shutdown();
      }

      // setup orb slam object
      pAgent = std::make_shared<ORB_SLAM3::System>(vocabulary_file_path, 
          settings_file_path,
          sensor_type,
          use_pangolin,
          0);
      image_scale = pAgent->GetImageScale();

      // create subscriptions
      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&MonoRealSense::image_callback, this, _1));

      // create timer
      timer = create_wall_timer(1s/30, std::bind(&MonoRealSense::timer_callback, this));
    }

    ~MonoRealSense()
    {
      if (use_live_feed) {
        vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetTrackedMapPoints();
        save_map_to_csv(map_points);
      }
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // change mono8 to something more? maybe orb_slam3 expects something else not sure
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);

      cv::Mat frame = cv_ptr->image;
      cv::imshow("image", frame);
      cv::waitKey(1);
      if(image_scale != 1.f) {
        cv::resize(frame, frame, cv::Size(frame.cols*image_scale, frame.rows*image_scale));
      }

      Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp);//, vImuMeas);
      std::string line;
      std::getline(imu_file, line);
      if (!Tcw.matrix().isIdentity() && count == 0)
      {
        std::istringstream ss(line);
        std::string token;
        std::vector<float> imu_data;
        while(std::getline(ss, token, ','))
        {
          imu_data.push_back(std::stof(token));
        }
        initial_orientation.x = imu_data[0];
        initial_orientation.y = imu_data[1];
        initial_orientation.z = imu_data[2];
        initial_orientation.w = imu_data[3];
        count++;
      }
      cv::imshow("frame", frame);
      cv::waitKey(1);
      timestamp += 1.0/30.0;
    }

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
        map_file << initial_orientation.x << "," << initial_orientation.y << "," << initial_orientation.z << "," << initial_orientation.w << std::endl;
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

    void timer_callback()
    {
      cv::Mat frame;
      input_video >> frame;
      if (!frame.empty()) {
        if(image_scale != 1.f) {
          cv::resize(frame, frame, cv::Size(frame.cols*image_scale, frame.rows*image_scale));
        }

        Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timestamp);//, vImuMeas);
        std::string line;
        std::getline(imu_file, line);
        if (!Tcw.matrix().isIdentity() && count == 0)
        {
          std::istringstream ss(line);
          std::string token;
          std::vector<float> imu_data;
          while(std::getline(ss, token, ','))
          {
            imu_data.push_back(std::stof(token));
          }
          initial_orientation.x = imu_data[0];
          initial_orientation.y = imu_data[1];
          initial_orientation.z = imu_data[2];
          initial_orientation.w = imu_data[3];
          count++;
        }
        cv::imshow("frame", frame);
        cv::waitKey(1);
        timestamp += 1.0/30.0;
      } else {
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "End of video file");
        vector<ORB_SLAM3::MapPoint*>map_points = pAgent->GetTrackedMapPoints();
        save_map_to_csv(map_points);
        pAgent->Shutdown();
        rclcpp::shutdown();
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::TimerBase::SharedPtr timer;

    sensor_msgs::msg::Imu imu_msg;

    bool use_live_feed;
    std::string video_name;
    cv::VideoCapture input_video;
    std::ifstream imu_file;

    std::shared_ptr<ORB_SLAM3::System> pAgent;
    std::string vocabulary_file_path;
    std::string settings_file_path;
    float image_scale;
    double timestamp;

    geometry_msgs::msg::Quaternion initial_orientation;
    int count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoRealSense>());
  rclcpp::shutdown();
  return 0;
}
