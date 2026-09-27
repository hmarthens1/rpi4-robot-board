# robot_vision

The robot's USB camera in C++: colour lane tracking (and following) with OpenCV, and
object detection with YOLOX on [ncnn](https://github.com/Tencent/ncnn). One node,
`vision_node`, owns the camera and does both, so frames never cross the network or
a process boundary.

Install on a robot (from the repo folder):

```bash
sudo bash scripts/install_vision.sh       # OpenCV (apt), ncnn (source, ~1 h once), models, service
journalctl -u robot-vision -f
```

## Why these choices

| Choice | Why |
|---|---|
| OpenCV **4.5.4 from Ubuntu** (`libopencv-dev`), not built from source | ROS 2 Humble's `cv_bridge` and image packages are built against exactly this version; a second OpenCV in `/usr/local` would give two ABIs in one process. Ubuntu's build already has V4L2, GStreamer, FFmpeg and NEON. The MSE112 labs build 4.10 from source because Raspberry Pi OS Bullseye ships an older OpenCV without ROS. |
| OpenCV, not a custom C image library | the Pi 4 has the memory (4 GB) and OpenCV's colour conversion, thresholding and resizing are NEON-vectorised: 640×480 lane tracking takes a few ms (see `vision_bench`). A hand-written library would be slower and untested. |
| **YOLOX nano/tiny on ncnn** | ncnn is the fastest CPU runtime for ARM; YOLOX is Apache-2.0 (Ultralytics YOLOv8/11 are AGPL-3.0) and is what the MSE112 project uses. Same models as the course (checked by SHA-256). |
| ncnn built from source at a pinned release | Ubuntu 22.04 doesn't package it. |

Alternatives worth trying later: NanoDet-Plus or YOLO-FastestV2 (faster, less accurate),
YOLOv8n/YOLO11n exported to ncnn (more accurate, AGPL), or a Coral/Hailo accelerator.

## Topics (in `/<robot>`)

| Topic | Type | |
|---|---|---|
| `vision/lane` | String (JSON) | `{"found", "offset" -1..1 (+ = right of centre), "angle_deg" (+ = heads right), "bands", "coverage", "fps"}` every frame |
| `vision/detections` | String (JSON) | `{"objects": [{"name", "prob", "x", "y", "w", "h"}], "ms", "model", "image_w", "image_h"}` |
| `vision/image/compressed` | CompressedImage | JPEG with overlays, 320 px wide, 3 Hz, only while someone subscribes |
| `vision/state` | String (JSON) | 1 Hz: camera, frame rates, all settings, last event |
| `vision/control` | String (JSON) | settings, below |
| `cmd_vel` | Twist | only while following a lane; `command_node` applies its limits |

```json
{"lane": true, "color": "yellow|blue|green|red|black|white", "hsv": [20, 80, 80, 35, 255, 255],
 "roi_top": 0.5, "two_lines": false,
 "detect": true, "model": "nano|tiny", "threshold": 0.4, "classes": ["cup", "person"],
 "follow": true, "speed": 0.3, "kp": 0.8, "ka": 0.5,
 "image": "annotated|mask|raw|off"}
```

```bash
ros2 topic echo /robot01/vision/lane
ros2 topic pub --once /robot01/vision/control std_msgs/String '{data: "{\"detect\": true}"}'
```

The dashboard's **Vision** tab shows the picture and has all these settings.

## Lane tracking

The image is shrunk to 320 px wide; the part below `roi_top` is blurred, converted to HSV and
thresholded for the colour; the mask is split into 4 horizontal bands, and in each band the
biggest blob's centre is a lane point (with `two_lines`, the middle between the leftmost and
rightmost blob). `offset` is the weighted mean of the points (lower bands count more);
`angle` is the slope of a straight line fitted through them.

Following: `turn = -(kp·offset + ka·angle)`, forward speed `speed·(1 - |turn|/2)`, both as
fractions of `command_node`'s `max_speed`. It is off at start and stops on a `stop` command
(the dashboard's and voice STOP), when the lane is lost for 1 s, or when the camera goes;
`command_node` still stops the robot 0.5 s after the last `cmd_vel` and at an obstacle.

Tune the colour with `"image": "mask"`: white is what matches.

## YOLOX

COCO's 80 classes. Detection runs on its own thread with 3 ncnn threads (leaving a core for
the camera, lane tracking and ROS), as fast as it can; it is off by default because it keeps
three cores busy. Measure on your Pi:

```bash
~/ros2_ws/install/robot_vision/lib/robot_vision/vision_bench
```

Model files: `/usr/local/share/robot_vision/models/yolox{N,T}.{param,bin}`, from
[Qengineering/YoloX-ncnn-Raspberry-Pi-4](https://github.com/Qengineering/YoloX-ncnn-Raspberry-Pi-4)
(BSD-3-Clause; YOLOX weights by Megvii, Apache-2.0).
