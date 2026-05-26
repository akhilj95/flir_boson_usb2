/*
 * Copyright (c) 2018 FLIR Systems, INC
 * Copyright (c) 2018-2019 AutonomouStuff, LLC
 * * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "flir_boson_usb2/BosonCamera.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace flir_boson_usb2
{

using namespace cv;

BosonCamera::BosonCamera(const rclcpp::NodeOptions & options)
: Node("boson_camera", options)
{
  frame_id_ = this->declare_parameter("frame_id", "boson_camera");
  dev_path_ = this->declare_parameter("dev", "/dev/video0");
  frame_rate_ = this->declare_parameter("frame_rate", 60.0);
  video_mode_str_ = this->declare_parameter("video_mode", "RAW16");
  zoom_enable_ = this->declare_parameter("zoom_enable", false);
  sensor_type_str_ = this->declare_parameter("sensor_type", "Boson_640");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");

  RCLCPP_INFO(this->get_logger(), "Initializing FLIR Boson on %s", dev_path_.c_str());

  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    [this]() {
      this->init_timer_->cancel(); // Prevent it from looping
      this->init();
    });
}

void BosonCamera::init()
{
  camera_info_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  image_pub_ = it_->advertiseCamera("image_raw", 1);

  if (video_mode_str_ == "RAW16") video_mode_ = RAW16;
  else if (video_mode_str_ == "YUV") video_mode_ = YUV;
  else {
    RCLCPP_ERROR(this->get_logger(), "Invalid video_mode value provided.");
    rclcpp::shutdown();
    return;
  }

  if (sensor_type_str_ == "Boson_320" || sensor_type_str_ == "boson_320") {
    sensor_type_ = Boson320;
    camera_info_->setCameraName("Boson320");
  } else if (sensor_type_str_ == "Boson_640" || sensor_type_str_ == "boson_640") {
    sensor_type_ = Boson640;
    camera_info_->setCameraName("Boson640");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid sensor_type value provided.");
    rclcpp::shutdown();
    return;
  }

  if (camera_info_->validateURL(camera_info_url_)) {
    camera_info_->loadCameraInfo(camera_info_url_);
  } else {
    RCLCPP_WARN(this->get_logger(), "camera_info_url could not be validated.");
  }

  if (!openCamera()) {
    rclcpp::shutdown();
    return;
  }

  auto period = std::chrono::duration<double>(1.0 / frame_rate_);
  capture_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&BosonCamera::captureAndPublish, this));
}

BosonCamera::~BosonCamera()
{
  closeCamera();
}

void BosonCamera::agcBasicLinear(const Mat& input_16, Mat* output_8, const int& height, const int& width)
{
  unsigned int max1 = 0;
  unsigned int min1 = 0xFFFF;
  unsigned int value1, value2, value3, value4;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      value1 = input_16.at<uchar>(i, j * 2 + 1) & 0xFF;
      value2 = input_16.at<uchar>(i, j * 2) & 0xFF;
      value3 = (value1 << 8) + value2;
      if (value3 <= min1) min1 = value3;
      if (value3 >= max1) max1 = value3;
    }
  }

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      value1 = input_16.at<uchar>(i, j * 2 + 1) & 0xFF;
      value2 = input_16.at<uchar>(i, j * 2) & 0xFF;
      value3 = (value1 << 8) + value2;
      value4 = (max1 - min1) == 0 ? 0 : ((255 * (value3 - min1))) / (max1 - min1);
      output_8->at<uchar>(i, j) = static_cast<uint8_t>(value4 & 0xFF);
    }
  }
}

bool BosonCamera::openCamera()
{
  if ((fd_ = open(dev_path_.c_str(), O_RDWR)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: Invalid Video Device.");
    return false;
  }

  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: Video Capture is not available.");
    return false;
  }

  struct v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (video_mode_ == RAW16) {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
    width_ = (sensor_type_ == Boson640) ? 640 : 320;
    height_ = (sensor_type_ == Boson640) ? 512 : 256;
  } else {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420;
    width_ = 640;
    height_ = 512;
  }

  format.fmt.pix.width = width_;
  format.fmt.pix.height = height_;

  if (ioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_S_FMT error. Format not supported.");
    return false;
  }

  struct v4l2_requestbuffers bufrequest;
  memset(&bufrequest, 0, sizeof(bufrequest));
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = 1;

  if (ioctl(fd_, VIDIOC_REQBUFS, &bufrequest) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_REQBUFS error.");
    return false;
  }

  memset(&bufferinfo_, 0, sizeof(bufferinfo_));
  bufferinfo_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo_.memory = V4L2_MEMORY_MMAP;
  bufferinfo_.index = 0;

  if (ioctl(fd_, VIDIOC_QUERYBUF, &bufferinfo_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_QUERYBUF error.");
    return false;
  }

  buffer_start_ = mmap(NULL, bufferinfo_.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, bufferinfo_.m.offset);
  if (buffer_start_ == MAP_FAILED) {
    RCLCPP_ERROR(this->get_logger(), "mmap error.");
    return false;
  }

  memset(buffer_start_, 0, bufferinfo_.length);

  int type = bufferinfo_.type;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_STREAMON error.");
    return false;
  }

  if (video_mode_ == RAW16) {
    thermal16_ = Mat(height_, width_, CV_16U, buffer_start_);
    thermal16_linear_ = Mat(height_, width_, CV_8U, 1);
  } else {
    int luma_height = height_ + height_ / 2;
    thermal_luma_ = Mat(luma_height, width_, CV_8UC1, buffer_start_);
    thermal_rgb_ = Mat(height_, width_, CV_8UC3, 1);
  }

  return true;
}

bool BosonCamera::closeCamera()
{
  if (fd_ >= 0) {
    int type = bufferinfo_.type;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
    close(fd_);
  }
  return true;
}

void BosonCamera::captureAndPublish()
{
  if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_QBUF error.");
    return;
  }

  if (ioctl(fd_, VIDIOC_DQBUF, &bufferinfo_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_DQBUF error.");
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = frame_id_;

  cv_bridge::CvImage cv_img;
  cv_img.header = header;

  if (video_mode_ == RAW16) {
    agcBasicLinear(thermal16_, &thermal16_linear_, height_, width_);

    if (!zoom_enable_) {
      Mat mask_mat, masked_img;
      threshold(thermal16_linear_, mask_mat, 0, 255, THRESH_BINARY | THRESH_OTSU);
      thermal16_linear_.copyTo(masked_img, mask_mat);

      Mat d_out_img, d_norm_image, gamma_corrected_image, d_gamma_corrected_image;
      double gamma = 0.8;
      masked_img.convertTo(d_out_img, CV_64FC1);
      normalize(d_out_img, d_norm_image, 0, 1, NORM_MINMAX, CV_64FC1);
      pow(d_out_img, gamma, d_gamma_corrected_image);
      d_gamma_corrected_image.convertTo(gamma_corrected_image, CV_8UC1);
      normalize(gamma_corrected_image, gamma_corrected_image, 0, 255, NORM_MINMAX, CV_8UC1);

      int erosion_size = 5;
      Mat top_hat_img, kernel = getStructuringElement(MORPH_ELLIPSE, Size(2 * erosion_size + 1, 2 * erosion_size + 1));
      morphologyEx(gamma_corrected_image, top_hat_img, MORPH_TOPHAT, kernel);

      cv_img.image = thermal16_linear_; // or top_hat_img if you prefer the filtered output
      cv_img.encoding = "mono8";
    } else {
      Size size(640, 512);
      resize(thermal16_linear_, thermal16_linear_zoom_, size);
      cv_img.image = thermal16_linear_zoom_;
      cv_img.encoding = "mono8";
    }
  } else {
    cvtColor(thermal_luma_, thermal_rgb_, COLOR_YUV2GRAY_I420, 0);
    cv_img.image = thermal_rgb_;
    cv_img.encoding = "mono8";
  }

  auto ci = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_->getCameraInfo());
  ci->header = header;

  image_pub_.publish(*cv_img.toImageMsg(), *ci);
}

}  // namespace flir_boson_usb2

RCLCPP_COMPONENTS_REGISTER_NODE(flir_boson_usb2::BosonCamera)
