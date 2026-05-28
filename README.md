# FLIR Boson USB ROS 2 Driver (`flir_boson_usb2`)

A ROS 2 (Humble) USB camera driver for the FLIR Boson thermal camera.

This package talks to the camera over the standard Linux V4L2 (`ioctl`) interface and publishes ROS 2 image topics via OpenCV and `image_transport`. The node is registered as an `rclcpp_components` composable node, so it can be loaded into a component container alongside other nodes to enable Intra-Process Communication (IPC) in multi-camera or downstream-processing pipelines.

Image topics are advertised with `rclcpp::SensorDataQoS` (best-effort, `KEEP_LAST` with shallow depth), appropriate for high-rate video where stale frames should be dropped rather than queued.

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
## Checking the Camera
Verify if the system detects the device
```bash
lsusb | grep FLIR
```

Many Boson models support UVC, so they work like webcam
```bash
v4l2-ctl --list-devices
```
It should return something like the following:
```bash
Boson: FLIR Video (usb-0000:00:14.0-7.3):
	/dev/video4 <-- This one should be used
	/dev/video5
	/dev/media2
```
## Launching the Camera

Use the provided Python launch file. Arguments can be overridden per-camera:

```bash
ros2 launch flir_boson_usb2 flir_boson.launch.py \
    dev:=/dev/video0 \
    video_mode:=YUV \
    frame_rate:=30.0
```

### The three video modes

The driver supports three `video_mode` values, each producing a different output stream:

- **`YUV`** (default) — The camera's internal DSP does AGC and produces a contrast-mapped 8-bit image. Published as `mono8`, or `bgr8` when `publish_color:=True`. Lowest host CPU cost, recommended for resource-constrained hosts such as a Raspberry Pi.

- **`RAW16`** — Raw 16-bit thermal counts from the bolometer array, published unchanged as `mono16`. No host-side processing. This is the mode to use for radiometric work, custom AGC, ML on raw thermal features, or anything else that needs the actual sensor values. RViz can display `mono16` directly (it auto-stretches).

- **`RAW16_AGC`** — Same source as `RAW16`, but the driver applies a host-side percentile-clipping AGC and publishes the result as `mono8`. Tunable via `raw16_agc_low_pct` / `raw16_agc_high_pct`. More robust to outlier pixels and isolated hot spots than the camera-internal AGC, at the cost of CPU time per frame.

### Notes on frame rate and performance

- **Hardware rate vs. driver polling.** The `frame_rate` argument controls the ROS 2 software polling timer, not the physical camera hardware. The driver uses a non-blocking `poll()` architecture, so setting the timer above the camera's actual rate (e.g. polling at 60 Hz on a 9 Hz export-restricted camera) is safe — `poll()` returns immediately when no frame is ready, and the published rate naturally settles at the hardware limit with negligible CPU overhead.

- **Automatic telemetry handling.** When hardware telemetry is enabled via the FLIR GUI, the camera appends one or more rows of binary metadata to the bottom of the frame. The driver negotiates the actual buffer dimensions with V4L2, processes the full buffer for unpacking, and then crops the published image back to the nominal sensor size (640×512 or 320×256) so calibration files and downstream nodes consuming `CameraInfo` continue to work unchanged. In `RAW16_AGC` mode, telemetry rows are also excluded from the histogram used for AGC, so they cannot skew contrast.

### Launch arguments

| Argument | Description | Default |
| :--- | :--- | :--- |
| `namespace` | ROS namespace for the camera node. | `flir_boson` |
| `frame_id` | TF frame ID stamped on each `Image` header. | `boson_camera` |
| `dev` | Linux video device path (e.g. `/dev/video4`). | `/dev/video0` |
| `frame_rate` | Polling rate for the V4L2 dequeue timer, in Hz. Any positive value is accepted; invalid values (≤ 0, NaN, inf) are clamped to 1.0 Hz with a warning. Typical hardware rates are 9, 30, or 60. | `30.0` |
| `video_mode` | `YUV`: camera-side AGC (`mono8`/`bgr8`, low CPU). `RAW16`: raw 16-bit thermal counts (`mono16`, no host processing). `RAW16_AGC`: host-side percentile AGC (`mono8`, tunable). | `YUV` |
| `publish_color` | If `True` and `video_mode:=YUV`, publish a `bgr8` colorized image instead of `mono8`. Ignored in `RAW16` and `RAW16_AGC` modes. | `False` |
| `raw16_agc_low_pct` | Bottom-tail clip percentage for `RAW16_AGC` (e.g. `1.0` discards the darkest 1% of pixels before linear stretch). Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` |
| `raw16_agc_high_pct` | Top-tail clip percentage for `RAW16_AGC` (e.g. `1.0` discards the brightest 1% of pixels before linear stretch). Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` |
| `zoom_enable` | Digital 2× upscale (`320×256` → `640×512`) of the published image. Only honored on `sensor_type:=Boson_320` in `RAW16_AGC` mode; ignored elsewhere. | `False` |
| `sensor_type` | Physical sensor array size. `Boson_320` or `Boson_640`. The driver cross-checks this against the V4L2-negotiated width and refuses to start on a mismatch. | `Boson_640` |
| `camera_info_url` | Camera calibration file URL (`file://` or `package://`). Empty publishes uncalibrated `CameraInfo`. See [Calibration](#calibration) below. | `""` |

### Tuning the RAW16_AGC percentiles

Defaults of `1.0 / 1.0` (discard 1% from each tail) work well for most scenes. Adjust if:

- The scene contains a small but very hot object (lamp, exhaust, sun) that's compressing the rest of the image — *raise* `raw16_agc_high_pct` (try `2.0` or `5.0`).
- The image looks washed-out or low-contrast — *lower* both percentages toward `0.5`.
- A few dead/stuck pixels are dominating the range — keep low percentages, even `0.1` is usually enough to discard isolated outliers thanks to the histogram-based clipping.

Setting both to `0.0` is equivalent to a pure min/max stretch.

## Published topics

- **`/<namespace>/image_raw`** (`sensor_msgs/msg/Image`) — The primary video stream. Encoding depends on `video_mode`:
  - `YUV` mode: `mono8`, or `bgr8` if `publish_color:=True`
  - `RAW16` mode: `mono16` (raw 16-bit thermal counts)
  - `RAW16_AGC` mode: `mono8`

- **`/<namespace>/camera_info`** (`sensor_msgs/msg/CameraInfo`) — Calibration matrices and metadata, populated from `camera_info_url` if provided.

Both topics are advertised with `rclcpp::SensorDataQoS` (best-effort). `image_transport` additionally exposes `image_raw/compressed`, `image_raw/compressedDepth`, and `image_raw/theora` topics if the corresponding plugins are installed.

## Calibration

This driver publishes empty `CameraInfo` by default. To load a calibration, set `camera_info_url`:

```bash
ros2 launch flir_boson_usb2 flir_boson.launch.py \
    camera_info_url:=file:///home/user/my_boson.yaml
```

The `example_calibrations/` folder in this package contains placeholder YAML files — they are **examples of the format**, not valid intrinsics for any specific camera. To generate a real calibration, see the ROS 2 [`camera_calibration`](https://docs.ros.org/en/humble/p/camera_calibration/) package.

`camera_info_url` accepts both `file://` URLs (absolute path on disk) and `package://` URLs (path relative to a ROS package share directory).

## Customizing the RAW16 pipeline

For users who want to apply their own filters, AGC curves, or feature detection on raw thermal counts, the `RAW16_AGC` branch in `BosonCamera::captureAndPublish()` is the place to start. It receives the cropped raw 16-bit buffer as a `cv::Mat` and runs `agc()` for percentile-clipping AGC. Replace or extend `agc()` with your own processing — anything from histogram equalization to top-hat filtering for hot-spot detection to neural-network inference — and the rest of the pipeline (zoom, publish, calibration metadata) keeps working unchanged.

If you'd rather not edit the driver, subscribe to a `RAW16` (`mono16`) stream from your own node and do the processing out-of-process. The `mono16` stream is the full 16-bit precision exactly as the sensor reports it.

## Troubleshooting

- **`ERROR: Invalid Video Device`** — Check `ls /dev/video*` and confirm your user is in the `video` group (`groups` should list it). A logout/login is required after `usermod`.

- **`VIDIOC_S_FMT error. Format not supported`** — The Boson typically exposes two `/dev/videoN` nodes (one for YUV, one for Y16). Try the next index, or confirm that the `video_mode` you requested matches what that node provides.

- **`Hardware mismatch! Configured for Boson_640 but V4L2 negotiated width 320`** — Wrong `sensor_type` for the physical camera. Set it to match the actual sensor.

- **`Driver reports YUV bytesperline=X but width=Y`** — The driver is reporting strided YUV buffers, which this node does not currently handle. File an issue with the output of `v4l2-ctl -d <dev> --get-fmt-video` attached.

- **`ros2 topic echo` or `rqt_image_view` shows nothing.** Most likely a QoS mismatch: the driver publishes best-effort, but many tools default to reliable. Either tell the subscriber to use best-effort (`ros2 topic echo /flir_boson/image_raw --qos-reliability best_effort`) or run `rqt_image_view` which negotiates QoS automatically.

- **`RAW16_AGC` output looks washed out or saturated.** Tune `raw16_agc_low_pct` and `raw16_agc_high_pct` — see [Tuning the RAW16_AGC percentiles](#tuning-the-raw16_agc-percentiles).

## Credits & lineage

This package is a ROS 2 Humble port and substantial rewrite of two earlier open-source projects. Credit to the original authors:

1. **[FLIR Systems / BosonUSB](https://github.com/FLIR/BosonUSB)** — Foundational V4L2 C++ interactions and 16-bit to 8-bit AGC conversions.
2. **[AutonomouStuff / flir_boson_usb](https://github.com/astuff/flir_boson_usb)** — Original ROS 1 wrapper, nodelet architecture, and RAW16 image processing filters.

Both original codebases and this ROS 2 port are released under the MIT License.