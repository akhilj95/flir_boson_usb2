# FLIR Boson USB ROS 2 Driver (`flir_boson_usb2`)

A ROS 2 (Humble) USB camera driver for the FLIR Boson thermal camera.

This package talks to the camera over the standard Linux V4L2 (`ioctl`) interface and publishes ROS 2 image topics via OpenCV and `image_transport`. The node is registered as an `rclcpp_components` composable node, so it can be loaded into a component container alongside other nodes to enable Intra-Process Communication (IPC) in multi-camera or downstream-processing pipelines.

Image topics are advertised with `rclcpp::SensorDataQoS` (best-effort, KEEP_LAST=1), appropriate for high-rate video where stale frames should be dropped rather than queued.

## Prerequisites

Your Linux environment needs `v4l-utils`, and your user must belong to the `video` group to access the USB interfaces.

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Install all dependencies (including v4l-utils and ROS packages)
rosdep install --from-paths src --ignore-src -r -y

# Grant your user access to USB Video devices
sudo usermod -aG video $USER
```

> You may need to log out and log back in for the group change to take effect.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select flir_boson_usb2 --symlink-install
source install/setup.bash
```

## Launching the Camera

Use the provided Python launch file. Arguments can be overridden per-camera:

```bash
ros2 launch flir_boson_usb2 flir_boson.launch.py \
    dev:=/dev/video0 \
    video_mode:=YUV \
    frame_rate:=30.0
```

### Notes on Frame Rate and Performance

- **Hardware rate vs. driver polling.** The `frame_rate` argument controls the ROS 2 software polling timer, not the physical camera hardware. The driver uses a non-blocking `poll()` architecture, so setting the timer above the camera's actual rate (e.g. polling at 60 Hz on a 9 Hz export-restricted camera) is safe — `poll()` returns immediately when no frame is ready, and the published rate naturally settles at the hardware limit with negligible CPU overhead.

- **Automatic telemetry handling.** When hardware telemetry is enabled via the FLIR GUI, the camera appends one or more rows of binary metadata to the bottom of the frame. The driver negotiates the actual buffer dimensions with V4L2, processes the full buffer for YUV unpacking, and then crops the published image back to the nominal sensor size (640×512 or 320×256) so that calibration files and downstream nodes consuming `CameraInfo` continue to work unchanged.

- **RAW16 vs YUV.** `video_mode:=RAW16` pushes raw 16-bit digital thermal counts over USB and runs a CPU-side pipeline (linear AGC → Otsu masking → gamma 0.8 → top-hat) to produce a viewable 8-bit image. `video_mode:=YUV` offloads contrast mapping to the camera's internal DSP and is recommended for resource-constrained hosts such as a Raspberry Pi.

### Launch Arguments

| Argument | Description | Default |
| :--- | :--- | :--- |
| `namespace` | ROS namespace for the camera node. | `flir_boson` |
| `frame_id` | TF frame ID stamped on each `Image` header. | `boson_camera` |
| `dev` | Linux video device path (e.g. `/dev/video4`). | `/dev/video0` |
| `frame_rate` | Polling rate for the V4L2 dequeue timer, in Hz. Any positive value is accepted; invalid values (≤ 0, NaN, inf) are clamped to 1.0 Hz with a warning. Typical hardware rates are 9, 30, or 60. | `30.0` |
| `video_mode` | `YUV` — camera-side AGC, low CPU. `RAW16` — host-side AGC pipeline (Otsu masking, gamma, top-hat), more CPU but more control. | `YUV` |
| `publish_color` | If `True` and `video_mode:=YUV`, publish a `bgr8` colorized image instead of `mono8`. Ignored in `RAW16` mode. | `False` |
| `zoom_enable` | Digital 2× upscale (`320×256` → `640×512`). Only meaningful for `sensor_type:=Boson_320` in `RAW16` mode. | `False` |
| `sensor_type` | Physical sensor array size. `Boson_320` or `Boson_640`. | `Boson_640` |
| `camera_info_url` | URL of the camera calibration YAML file (e.g. `file://path/to/Boson640.yaml`). Empty disables loading. | `""` |

## Published Topics

- **`/<namespace>/image_raw`** (`sensor_msgs/msg/Image`)
  The primary video stream. Encoding is `mono8` by default, or `bgr8` when `publish_color:=True` in YUV mode.

- **`/<namespace>/camera_info`** (`sensor_msgs/msg/CameraInfo`)
  Calibration matrices and metadata, populated from `camera_info_url` if provided.

Both topics are advertised with `rclcpp::SensorDataQoS` (best-effort).

## Troubleshooting

- **`ERROR: Invalid Video Device`** — Check `ls /dev/video*` and confirm your user is in the `video` group (`groups` should list it). A logout/login is required after `usermod`.
- **`VIDIOC_S_FMT error. Format not supported`** — The Boson typically exposes two `/dev/videoN` nodes (one for YUV, one for Y16). Try the next index, or check that your `video_mode` matches what that node provides.
- **`Hardware mismatch! Configured for Boson_640 but V4L2 negotiated width 320`** — Wrong `sensor_type` for the physical camera. Set it to match the actual sensor.
- **`Driver reports YUV bytesperline=X but width=Y`** — The driver is reporting strided YUV buffers, which this node does not currently handle. File an issue with the output of `v4l2-ctl -d <dev> --get-fmt-video` attached.

## Credits & Lineage

This ROS 2 package is a direct descendant and port of two open-source projects. Immense credit goes to the original authors:

1. **[FLIR Systems / BosonUSB](https://github.com/FLIR/BosonUSB)** — Foundational V4L2 C++ interactions and 16-bit to 8-bit AGC conversions.
2. **[AutonomouStuff / flir_boson_usb](https://github.com/astuff/flir_boson_usb)** — Original ROS 1 wrapper, nodelet architecture, and RAW16 image processing filters.

Both original codebases and this ROS 2 port are released under the MIT License.