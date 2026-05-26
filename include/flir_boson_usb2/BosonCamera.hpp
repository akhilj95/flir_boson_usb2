#ifndef FLIR_BOSON_USB2_BOSONCAMERA_HPP
#define FLIR_BOSON_USB2_BOSONCAMERA_HPP

#include <string>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace flir_boson_usb2
{

enum Encoding { YUV = 0, RAW16 = 1 };
enum SensorTypes { Boson320, Boson640 };

class BosonCamera : public rclcpp::Node
{
public:
  explicit BosonCamera(const rclcpp::NodeOptions & options);
  ~BosonCamera() override;

private:
  void init();
  void agcBasicLinear(const cv::Mat& input_16, cv::Mat* output_8, const int& height, const int& width);
  bool openCamera();
  bool closeCamera();
  void captureAndPublish();

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Hardware variables
  int32_t width_, height_, fd_;
  struct v4l2_capability cap_;
  struct v4l2_buffer bufferinfo_;
  void* buffer_start_;
  
  // OpenCV Mats
  cv::Mat thermal16_, thermal16_linear_, thermal16_linear_zoom_, thermal_luma_, thermal_rgb_;

  // Parameters
  std::string frame_id_, dev_path_, camera_info_url_, video_mode_str_, sensor_type_str_;
  double frame_rate_;
  Encoding video_mode_;
  bool zoom_enable_;
  SensorTypes sensor_type_;
};

}  // namespace flir_boson_usb2

#endif  // FLIR_BOSON_USB2_BOSONCAMERA_HPP
