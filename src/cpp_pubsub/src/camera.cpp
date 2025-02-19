/*
Copyright (c) 2019 Andreas Klintberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/string.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std::chrono_literals;

class CameraDriver : public rclcpp::Node {
public:
  // ! This is the site of an undefined error during make_shared call
  CameraDriver() : Node("usb_camera_driver") {
    frame_id_ = this->declare_parameter("frame_id", "camera");

    image_width = this->declare_parameter("image_width", 1280);
    image_height = this->declare_parameter("image_height", 720);
    fps = this->declare_parameter("fps", 10.0);

    camera_id = this->declare_parameter("camera_id", 0);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    camera_info_pub =
        image_transport::CameraPublisher(this, "image", custom_qos_profile);

    camera_manager =
        std::make_shared<camera_info_manager::CameraInfoManager>(this);

    /* get ROS2 config parameter for camera calibration file */
    auto camera_calibration_file_param_ = this->declare_parameter(
        "camera_calibration_file", "file://config/camera.yaml");
    camera_manager->loadCameraInfo(camera_calibration_file_param_);

    cap.open(camera_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

    last_frame = std::chrono::steady_clock::now();

    timer = this->create_wall_timer(
        1ms, std::bind(&CameraDriver::ImageCallback, this));
  };

private:
  rclcpp::TimerBase::SharedPtr timer;

  cv::Mat frame;
  cv::Mat flipped_frame;
  cv::VideoCapture cap;

  bool is_flipped;

  std::shared_ptr<sensor_msgs::msg::Image> image;
  std::string frame_id_;

  int image_height;
  int image_width;
  int camera_id;
  int fps;

  std::chrono::steady_clock::time_point last_frame;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_manager;
  image_transport::CameraPublisher camera_info_pub;

  std::shared_ptr<sensor_msgs::msg::Image>
  ConvertFrameToMessage(cv::Mat &frame) {
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    // Make sure output in the size the user wants even if it is not native
    if (frame.rows != image_width || frame.cols != image_height) {
      cv::resize(frame, frame, cv::Size(image_width, image_height));
    }

    /* To remove CV-bridge and boost-python3 dependencies, this is pretty much a
     * copy of the toImageMsg method in cv_bridge. */
    ros_image.header = header_;
    ros_image.height = frame.rows;
    ros_image.width = frame.cols;
    ros_image.encoding = "bgr8";
    /* FIXME c++20 has std::endian */
    // ros_image.is_bigendian = (std::endian::native == std::endian::big);
    ros_image.is_bigendian = false;
    ros_image.step = frame.cols * frame.elemSize();
    size_t size = ros_image.step * frame.rows;
    ros_image.data.resize(size);

    if (frame.isContinuous()) {
      memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
    } else {
      // Copy by row by row
      uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
      uchar *cv_data_ptr = frame.data;
      for (int i = 0; i < frame.rows; ++i) {
        memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
        ros_data_ptr += ros_image.step;
        cv_data_ptr += frame.step;
      }
    }

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
  };
  void ImageCallback() {
    cap >> frame;

    auto now = std::chrono::steady_clock::now();

    if (!frame.empty() &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame)
                .count() > 1 / fps * 1000) {
      last_frame = now;

      // Convert to a ROS2 image
      if (!is_flipped) {
        image = ConvertFrameToMessage(frame);
      } else {
        // Flip the frame if needed
        cv::flip(frame, flipped_frame, 1);
        image = ConvertFrameToMessage(frame);
      }

      // Put the message into a queue to be processed by the middleware.
      // This call is non-blocking.
      sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
          new sensor_msgs::msg::CameraInfo(camera_manager->getCameraInfo()));

      rclcpp::Time timestamp = this->get_clock()->now();

      image->header.stamp = timestamp;
      image->header.frame_id = frame_id_;

      camera_info_msg_->header.stamp = timestamp;
      camera_info_msg_->header.frame_id = frame_id_;

      camera_info_pub.publish(image, camera_info_msg_);
    }
  };
};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(CameraDriver)
