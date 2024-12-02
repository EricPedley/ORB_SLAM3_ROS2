#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <tuple>

namespace py = pybind11;

struct Object {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointXYZ centroid;
  std::string label;
  float confidence;
};

struct Result {
  bool success = false;
  std::string timestamp = "";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  std::list<std::tuple<cv::Mat, cv::Mat, Sophus::SE3f,
                       std::map<std::pair<int, int>, int>>>
    mapping_data;
};

struct BoundingBox {
  int x1, y1, x2, y2;
  BoundingBox(int x1, int y1, int x2, int y2) : x1(x1), y1(y1), x2(x2), y2(y2)
  {
  }
};

template <typename T> class SemanticMapping {
public:
  pcl::PointXYZ calculate_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    pcl::PointXYZ centroid;
    for (const auto &point : cloud->points) {
      centroid.x += point.x;
      centroid.y += point.y;
      centroid.z += point.z;
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->points.resize(cloud->width * cloud->height);

    centroid.x /= cloud->size();
    centroid.y /= cloud->size();
    centroid.z /= cloud->size();

    return centroid;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_from_bounding_box(
    std::tuple<std::string, float, BoundingBox> bounding_box,
    std::map<std::pair<int, int>, int> pixel_to_point_map,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Sophus::SE3f pose)
  {
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<> dis;
    gen.seed(rd());

    std::string label = std::get<0>(bounding_box);
    BoundingBox box = std::get<2>(bounding_box);
    int r = dis(gen);
    int g = dis(gen);
    int b = dis(gen);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB closest_point;
    float l2_closest = 0.0;
    // figure out what the closest point in the pointcloud is to the camera
    // and add all points to the object cloud
    for (int y = box.y1; y < box.y2; ++y) {
      for (int x = box.x1; x < box.x2; ++x) {
        if (pixel_to_point_map.find(std::make_pair(y, x)) !=
            pixel_to_point_map.end()) {
          int index = pixel_to_point_map[std::make_pair(y, x)];
          pcl::PointXYZRGB point = cloud->points[index];

          if (object_cloud->empty()) {
            closest_point = point;
            l2_closest = (point.x - pose.translation().x()) *
                           (point.x - pose.translation().x()) +
                         (point.y - pose.translation().y()) *
                           (point.y - pose.translation().y()) +
                         (point.z - pose.translation().z()) *
                           (point.z - pose.translation().z());
          }

          float l2 = (point.x - pose.translation().x()) *
                       (point.x - pose.translation().x()) +
                     (point.y - pose.translation().y()) *
                       (point.y - pose.translation().y()) +
                     (point.z - pose.translation().z()) *
                       (point.z - pose.translation().z());

          if (l2 < l2_closest) {
            closest_point = point;
            l2_closest = l2;
          }

          point.r = r;
          point.g = g;
          point.b = b;
          object_cloud->push_back(point);
        }
      }
    }

    if (object_cloud->empty()) {
      return object_cloud;
    }

    // filter out the points that are too far away from the closest point to
    // the camera
    float threshold = 1.0;
    object_cloud->points.erase(
      std::remove_if(
        object_cloud->points.begin(), object_cloud->points.end(),
        [&closest_point, threshold](const pcl::PointXYZRGB &point) {
          float l2 = std::sqrt(std::pow(point.x - closest_point.x, 2) +
                               std::pow(point.y - closest_point.y, 2) +
                               std::pow(point.z - closest_point.z, 2));
          return l2 > threshold;
        }),
      object_cloud->points.end());

    object_cloud->width = object_cloud->points.size();
    object_cloud->height = 1;

    return object_cloud;
  }

  std::pair<cv::Mat, std::map<std::pair<int, int>, int>>
  project_cloud_to_camera(const cv::Mat &image,
                          const cv::Mat &camera_matrix,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          const Sophus::SE3f &camera_transform)
  {
    // assert(!camera_transform.isNull());
    assert(!cloud->empty());
    assert(camera_matrix.type() == CV_64FC1 && camera_matrix.cols == 3 &&
           camera_matrix.cols == 3);

    float fx = camera_matrix.at<double>(0, 0);
    float fy = camera_matrix.at<double>(1, 1);
    float cx = camera_matrix.at<double>(0, 2);
    float cy = camera_matrix.at<double>(1, 2);

    cv::Mat registered = cv::Mat::zeros(image.size(), CV_32FC1);
    Sophus::SE3f t = camera_transform.inverse();

    // create a map from each pixel to the index of their point in the
    // pointcloud
    std::map<std::pair<int, int>, int> pixel_to_point_map;

    int count = 0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin();
         it != cloud->end(); ++it) {
      pcl::PointXYZRGB ptScan = *it;
      // transform the current point into the camera frame
      Eigen::Vector3f transformed_point =
        t * Eigen::Vector3f(ptScan.x, ptScan.y, ptScan.z);

      pcl::PointXYZRGB result = ptScan; // Copy other fields like RGB
      result.x = transformed_point.x();
      result.y = transformed_point.y();
      result.z = transformed_point.z();

      // re-project in camera frame
      float z = ptScan.z;
      bool set = false;
      if (z > 0.0f) {
        float invZ = 1.0f / z;
        float dx = (fx * ptScan.x) * invZ + cx;
        float dy = (fy * ptScan.y) * invZ + cy;
        int dx_low = dx;
        int dy_low = dy;
        int dx_high = dx + 0.5f;
        int dy_high = dy + 0.5f;
        // return uIsFinite(value) && !(value < low) && !(value >= high);

        if ((dx_low > 0 && dx_low <= registered.cols) &&
            (dy_low > 0 && dy_low <= registered.rows)) {
          set = true;
          float &zReg = registered.at<float>(dy_low, dx_low);
          // color the point in the point cloud with the pixel color
          it->r = image.at<cv::Vec3b>(dy_low, dx_low)[2];
          it->g = image.at<cv::Vec3b>(dy_low, dx_low)[1];
          it->b = image.at<cv::Vec3b>(dy_low, dx_low)[0];
          if (zReg == 0 || z < zReg) {
            zReg = z;
          }
        }
        if ((dx_low != dx_high || dy_low != dy_high) &&
            (dx_high > 0 && dx_high <= registered.cols) &&
            (dy_high > 0 && dy_high <= registered.rows)) {
          set = true;
          float &zReg = registered.at<float>(dy_high, dx_high);
          it->r = image.at<cv::Vec3b>(dy_low, dx_low)[2];
          it->g = image.at<cv::Vec3b>(dy_low, dx_low)[1];
          it->b = image.at<cv::Vec3b>(dy_low, dx_low)[0];
          if (zReg == 0 || z < zReg) {
            zReg = z;
          }
        }
        if (set) {
          pixel_to_point_map[std::make_pair(dy_low, dx_low)] = count;
          count++;
        }
      }
    }
    std::cout << "Points in camera=" << count << "/" << cloud->points.size()
              << std::endl;

    return {registered, pixel_to_point_map};
  }

  cv::Mat numpy_to_mat(const py::array_t<uint8_t> &np_array)
  {
    py::buffer_info buf = np_array.request();
    cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC3, (uchar *)buf.ptr);
    return mat;
  }

  py::array mat_to_numpy(const cv::Mat &mat)
  {
    return py::array_t<uint8_t>({mat.rows, mat.cols, mat.channels()},
                                {mat.step[0], mat.step[1], sizeof(uint8_t)},
                                mat.data);
  }

  std::vector<Object> semantic_mapping(
    py::object &net,
    std::list<std::tuple<cv::Mat, cv::Mat, Sophus::SE3f,
                         std::map<std::pair<int, int>, int>>> &mapping_data,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string &timestamp)
  {
    // start by getting the detections, and creating some pretty images
    std::vector<Object> objects; // cloud, closest point, label, confidence
    std::unordered_map<std::string, int> object_counts;
    std::vector<cv::Mat> detection_frames;
    int iter = 0;
    for (const auto &frame : mapping_data) {
      cv::Mat rgb = std::get<0>(frame);
      cv::Mat depth = std::get<1>(frame);
      Sophus::SE3f pose = std::get<2>(frame);
      std::map<std::pair<int, int>, int> pixel_to_point_map =
        std::get<3>(frame);
      py::array np_array = mat_to_numpy(rgb);
      py::list detections = net.attr("predict")(np_array);

      std::vector<std::tuple<std::string, float, BoundingBox>> bounding_boxes;
      for (auto detection : detections) {
        py::object boxes = detection.attr("boxes");
        py::object names = detection.attr("names");
        py::object speed = detection.attr("speed");
        if (!boxes.is_none()) {
          auto box_list =
            boxes.attr("xyxy")
              .cast<py::list>(); // Example of accessing box coordinates
          for (size_t i = 0; i < py::len(box_list); ++i) {
            py::object box = box_list[i];
            py::object conf_tensor = boxes.attr("conf");
            if (conf_tensor[py::int_(i)].cast<float>() < 0.8) {
              continue;
            }

            // Extract the box coordinates
            auto numpy_array =
              box_list[py::int_(0)].attr("cpu")().attr("numpy")();

            // Access individual elements using NumPy indexing.
            int x1 = numpy_array[py::int_(0)].cast<int>();
            int y1 = numpy_array[py::int_(1)].cast<int>();
            int x2 = numpy_array[py::int_(2)].cast<int>();
            int y2 = numpy_array[py::int_(3)].cast<int>();

            // std::cout << "Box: " << x1 << ", " << y1 << ", " << x2 << ", "
            // << y2
            //           << std::endl;

            cv::Mat detection_frame = rgb.clone();

            // Draw the box on the image
            cv::rectangle(detection_frame, cv::Point(x1, y1), cv::Point(x2, y2),
                          cv::Scalar(0, 255, 0), 2);

            // Add the label to the image
            py::object names = detection.attr("names");
            py::object classes = boxes.attr("cls");
            std::string label =
              names[py::int_(classes[py::int_(i)])].cast<std::string>();
            cv::putText(detection_frame, label, cv::Point(x1, y1 - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0),
                        2);

            // Add the confidence to the image
            float confidence = conf_tensor[py::int_(i)].cast<float>();
            cv::putText(detection_frame, std::to_string(confidence),
                        cv::Point(x1, y1 - 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 0), 2);

            // add the bounding box with the label to the list for later
            if (label == "chair" || label == "cup") {
              bounding_boxes.push_back(
                {label, confidence, BoundingBox(x1, y1, x2, y2)});
            } else {
              continue;
            }

            // Add the speed to the image
            py::object speed_py = detection.attr("speed");
            std::string speed = py::str(speed_py).cast<std::string>();
            cv::putText(detection_frame, speed, cv::Point(x1, y1 - 50),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0),
                        2);

            // Add the timestamp to the image
            cv::putText(detection_frame, timestamp, cv::Point(x1, y1 - 70),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0),
                        2);

            // Display the image
            // cv::imshow("Image", detection_frame);
            // cv::imshow("Depth", depth);
            // cv::waitKey(1);

            detection_frames.push_back(detection_frame);
          }
        }
      }
      // cv::destroyAllWindows();

      std::vector<Object> new_objects;
      for (const auto &elem : bounding_boxes) {
        std::string label = std::get<0>(elem);
        float conf = std::get<1>(elem);
        auto detection = detection_frames.at(iter);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud =
          object_cloud_from_bounding_box(elem, pixel_to_point_map, cloud, pose);

        std::cout << "object cloud size: " << object_cloud->size() << std::endl;
        std::cout << "Detected " << label << " with confidence " << conf
                  << std::endl;
        cv::imshow("Detection", detection_frames.at(iter));
        cv::imshow("depth", depth);
        cv::waitKey(0);
        // calculate the centroid of the object's pointcloud
        pcl::PointXYZ centroid = calculate_centroid(object_cloud);

        // figure out if we've already seen this object before, and if we have
        // then add its points to the pointcloud, recalculate the centroid,
        // and update the label and confidence with the highest confidence
        // value.
        if (objects.empty()) {
          objects.push_back({object_cloud, centroid, label, conf});
        } else {
          bool found = false;
          for (auto &elem : objects) {
            float l2 = std::sqrt(std::pow((elem.centroid.x - centroid.x), 2) +
                                 std::pow((elem.centroid.y - centroid.y), 2) +
                                 std::pow((elem.centroid.z - centroid.z), 2));
            if (l2 < 1.0) {
              elem.label = conf > elem.confidence ? label : elem.label;
              elem.confidence = conf > elem.confidence ? conf : elem.confidence;
              found = true;
            }
          }
          if (!found) {
            objects.push_back({object_cloud, centroid, label, conf});
            new_objects.push_back({object_cloud, centroid, label, conf});
            object_counts[label]++;
          }
        }
        iter++;
      }
      std::string file_path = std::string(PROJECT_PATH) + "/output/" +
                              timestamp + "/landmarks/" + timestamp + ".yaml";
      YAML::Node node;

      for (const auto &object : new_objects) {
        if (object.cloud->size() < 5) {
          std::cout << "Object " << object.label
                    << " has less than 5 points, skipping. Size: "
                    << object.cloud->size() << std::endl;
          std::cout << "centroid: " << object.centroid.x << ", "
                    << object.centroid.y << ", " << object.centroid.z
                    << std::endl;
          continue;
        }
        std::string path = std::string(PROJECT_PATH) + "/output/" + timestamp +
                           "/objects/" + object.label +
                           std::to_string(object_counts[object.label]) + ".pcd";
        pcl::io::savePCDFileBinary(path, *object.cloud);

        if (std::filesystem::exists(file_path)) {
          node = YAML::LoadFile(file_path);
        }

        // Save the new landmark's name and coordinates
        node[object.label + std::to_string(object_counts[object.label])]["x"] =
          object.centroid.x;
        node[object.label + std::to_string(object_counts[object.label])]["y"] =
          object.centroid.y;

        // Open the file for writing
        std::ofstream file(file_path);

        // Write to the file and close it
        file << node;
        file.close();
      }
    }
    iter = 0;
    std::string path =
      std::string(PROJECT_PATH) + "/output/" + timestamp + "/detections/";
    std::cout << "saving images to: " << path << std::endl;
    for (const auto &frame : detection_frames) {
      cv::Mat detection_frame = frame;

      // save the image
      if (std::filesystem::exists(path)) {
        cv::imwrite(path + "detection" + std::to_string(iter) + ".png",
                    detection_frame);
      }
      iter++;
    }
    std::cout << "Finished semantic mapping" << std::endl;
    return objects;
  }

private:
};
