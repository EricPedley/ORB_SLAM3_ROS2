#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <apriltag/apriltag.h>


#include <opencv2/opencv.hpp>

#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class AprilTagNode : public rclcpp::Node
{
  public:
    AprilTagNode()
    : Node("apriltag_node")
    {
      // declare parameters
      declare_parameter("apriltag_family", "tagStandard41h12");

      // get parameters
      apriltag_family = get_parameter("apriltag_family").as_string();

      // create apriltag family
      if (apriltag_family == "tag16h5")
      {
        tf = tag16h5_create();
      }
      else if (apriltag_family == "tag25h9")
      {
        tf = tag25h9_create();
      }
      else if (apriltag_family == "tag36h10")
      {
        tf = tag36h10_create();
      }
      else if (apriltag_family == "tag36h11")
      {
        tf = tag36h11_create();
      }
      else if (apriltag_family == "tagCircle21h7")
      {
        tf = tagCircle21h7_create();
      }
      else if (apriltag_family == "tagCircle49h12")
      {
        tf = tagCircle49h12_create();
      }
      else if (apriltag_family == "tagCustom48h12")
      {
        tf = tagCustom48h12_create();
      }
      else if (apriltag_family == "tagStandard41h12")
      {
        tf = tagStandard41h12_create();
      }
      else if (apriltag_family == "tagStandard52h13")
      {
        tf = tagStandard52h13_create();
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Invalid apriltag family");
      }

      // create apriltag detector
      detector = apriltag_detector_create();
      tf = tagStandard41h12_create();
      apriltag_detector_add_family(detector, tf);

      // declare subscribers
      image_sub = create_subscription<sensor_msgs::msg::Image>(
          "camera/color/image_raw", 10, std::bind(&AprilTagNode::image_callback, this, _1));
    }

    ~AprilTagNode()
    {
      apriltag_detector_destroy(detector);
      if (apriltag_family == "tag16h5")
      {
        tag16h5_destroy(tf);
      }
      else if (apriltag_family == "tag25h9")
      {
        tag25h9_destroy(tf);
      }
      else if (apriltag_family == "tag36h10")
      {
        tag36h10_destroy(tf);
      }
      else if (apriltag_family == "tag36h11")
      {
        tag36h11_destroy(tf);
      }
      else if (apriltag_family == "tagCircle21h7")
      {
        tagCircle21h7_destroy(tf);
      }
      else if (apriltag_family == "tagCircle49h12")
      {
        tagCircle49h12_destroy(tf);
      }
      else if (apriltag_family == "tagCustom48h12")
      {
        tagCustom48h12_destroy(tf);
      }
      else if (apriltag_family == "tagStandard41h12")
      {
        tagStandard41h12_destroy(tf);
      }
      else if (apriltag_family == "tagStandard52h13")
      {
        tagStandard52h13_destroy(tf);
      }
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat frame = cv_ptr->image;

      image_u8_t im = {cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.data};
      zarray_t *detections = apriltag_detector_detect(detector, &im);
      RCLCPP_INFO_STREAM(get_logger(), "Detected " << zarray_size(detections) << " tags");
      for (int i = 0; i < zarray_size(detections); i++)
      {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        RCLCPP_INFO_STREAM(get_logger(), "Detected tag " << det->id);
        RCLCPP_INFO_STREAM(get_logger(), "Detected tag center " << det->c[0] << ", " << det->c[1]);
      }

      // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Scalar(0, 0xff, 0), 2);
            line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0, 0, 0xff), 2);
            line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Scalar(0xff, 0, 0), 2);
            line(frame, cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0xff, 0, 0), 2);

            std::stringstream ss;
            ss << det->id;
            cv::String text = ss.str();
            int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, cv::Point(det->c[0]-textsize.width/2.0,
                                       det->c[1]+textsize.height/2.0),
                    fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);

        cv::imshow("Tag Detections", frame);
        cv::waitKey(1);

    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    std::string apriltag_family;
    apriltag_family_t *tf;
    apriltag_detector_t *detector;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagNode>());
  rclcpp::shutdown();
  return 0;
}
