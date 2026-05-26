# FLIR Boson USB ROS 2 Driver (`flir_boson_usb2`)

A ROS 2 (Humble) USB camera driver for the FLIR Boson thermal camera.

This package interacts with the camera using the standard Linux V4L2 (`ioctl`) interface and publishes ROS 2 image topics using OpenCV and `image_transport`.

It is built utilizing ROS 2 `rclcpp_components` to allow for zero-copy memory transport (Intra-Process Communication) in complex, multi-camera environments.

## Prerequisites

To use this driver, your Linux environment must have `v4l-utils` installed, and your user must belong to the `video` group to access the USB interfaces.

Run the following commands to install dependencies and grant permissions:

```bash
sudo apt-get install v4l-utils ros-humble-camera-info-manager ros-humble-image-transport-plugins
sudo usermod -aG video $USER
```
Note: You may need to log out and log back in for the group changes to take effect.

## Launching the Camera

Use the provided Python launch file to start the node. You can dynamically pass arguments for your specific hardware setup:

```bash
ros2 launch flir_boson_usb2 flir_boson.launch.py \
    dev:=/dev/video0 \
    video_mode:=YUV
```
### Launch Arguments
| Argument | Description | Default |
| :--- | :--- | :--- |
| `namespace` | The ROS namespace for the camera nodes. | `flir_boson` |
| `frame_id` | The TF frame ID for the camera. | `boson_camera` |
| `dev` | The Linux file descriptor location for the camera (e.g., `/dev/video4`). | `/dev/video0` |
| `frame_rate` | Target polling rate for the V4L2 USB bus. Valid values are `30.0` or `60.0`. | `30.0` |
| `video_mode` | The hardware output mode of the camera.<br><br>**`YUV`:** Offloads AGC and contrast mapping to the camera's internal hardware DSP. Output is a clean, low-CPU overhead stream.<br>**`RAW16`:** Pushes raw 16-bit digital thermal counts over USB. The node applies a heavy CPU-based software filter pipeline (Otsu masking, Gamma=0.8, Top-Hat filter) to format the image for viewing. | `YUV` |
| `zoom_enable` | Digital zoom flag. Valid values are `True` or `False`. | `False` |
| `sensor_type` | The size of your physical sensor array. Options are `Boson_320` or `Boson_640`. | `Boson_640` |
| `camera_info_url` | Location of the camera calibration file. | `file://<package_dir>/example_calibrations/Boson640.yaml` |

## Published Topics
* `/flir_boson/image_raw (sensor_msgs/msg/Image)`
 The primary video stream, published as an 8-bit grayscale (mono8) image.

* `/flir_boson/camera_info (sensor_msgs/msg/CameraInfo)`
 Camera calibration matrices and metadata.

## Credits & Lineage

This ROS 2 package is a direct descendant and port of two open-source projects. Immense credit goes to the original authors:

1. **[FLIR Systems / BosonUSB](https://github.com/FLIR/BosonUSB)**: The foundational V4L2 C++ interactions and 16-bit to 8-bit AGC conversions were developed directly by FLIR Systems.

2. **[AutonomouStuff / flir_boson_usb](https://github.com/astuff/flir_boson_usb)**: The original ROS 1 wrapper, nodelet architecture, and advanced RAW16 image processing filters were developed by the AutonomouStuff Software Development Team.

Both original codebases and this ROS 2 port are provided under the MIT License.