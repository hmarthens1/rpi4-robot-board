# robot_vision

The robot's USB camera in C++: colour lane tracking (and following) with OpenCV, and
object detection with YOLOX on [ncnn](https://github.com/Tencent/ncnn). One node,
`vision_node`, owns the camera and does both, so frames never cross the network or
a process boundary.

Install on a robot (from the repo folder):

```bash
sudo bash scripts/install_vision.sh       # OpenCV (apt), ncnn (source, ~12 min once), models, service
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
| `cmd_vel` | Twist | only while searching for or following a lane; `command_node` applies its limits |
| `command` | String (JSON) | out while searching: `arm_servos` to put the arm in the look pose and pan the base servo |

```json
{"lane": true, "color": "yellow|blue|green|red|black|white", "hsv": [20, 80, 80, 35, 255, 255],
 "roi_top": 0.5, "two_lines": false,
 "detect": true, "model": "nano|tiny", "threshold": 0.4, "classes": ["cup", "person"],
 "follow": true, "search": true, "look": {"1": 2500, "3": 725, "4": 2444, "5": 1486, "6": 1432}, "speed": 0.7, "min_drive": 0.6,
 "kp": 0.8, "ka": 0.5,
 "image": "annotated|mask|raw|off"}
```

```bash
ros2 topic echo /robot01/vision/lane
ros2 topic pub --once /robot01/vision/control std_msgs/String '{data: "{\"detect\": true}"}'
```

The dashboard's **Vision** tab shows the picture and has all these settings.

## Lane tracking

The image is shrunk to 320 px wide; the part below `roi_top` is blurred and matched for the
colour (below); the mask is split into 4 horizontal bands, and in each band the
biggest blob's centre is a lane point (with `two_lines`, the middle between the leftmost and
rightmost blob). `offset` is the weighted mean of the points (lower bands count more);
`angle` is the slope of a straight line fitted through them.

**Colour.** yellow, green, blue and red are matched in Lab *relative to the floor*: a pixel
matches when it is at least `min_chroma` more of that colour than the median of the search area
(the floor), within a hue angle band. On robot01 the camera over-exposes pale yellow tape on the
grey floor (V 250, S 30-65), and the old fixed HSV threshold (S >= 80) found nothing - the first
live follow stopped after 1 s with "lane lost" while the tape was in the middle of the picture
(`test/data/robot01_yellow_tape.jpg`, now a regression test). The median is the floor as long as
the tape covers less than half of the search area. black, white and custom `hsv` ranges still use
HSV thresholds. Costs 5.4 ms/frame on the Pi 4 (HSV: 2.8 ms). Tune with `"image": "mask"`: white
is what matches.

**Search, then follow** (`search`: true, the default; `lane_seeker.cpp`). robot01's camera is on
the arm, so it can look around without moving the robot:

1. the arm goes to the `look` pose (robot01's, set by hand: gripper 2500, wrist 725, elbow 2444,
   shoulder 1486, base 1432; set your own with `"look": {"1": .., "3": .., "4": .., "5": .., "6": ..}`,
   or `false` to leave the arm where it is). Its base pulse is "straight ahead";
2. the base servo pans 0, +17, -17, +34, -34, +51, -51 deg around it (the side the lane was last seen
   first); at each step it waits 0.7 s and needs the lane in 3 of 4 frames;
3. found: bearing = pan angle - atan(offset · 0.584); the robot turns in place by that much
   (150 deg/s at full command) while the camera pans back to the centre, then looks again;
4. ahead within 12 deg with the camera centred: follow. Nothing in a whole sweep: turn the
   robot 90 deg and sweep again, up to a full turn, then give up ("no lane found all around");
5. while following, a lane lost for 1 s starts a new search instead of stopping.

**Following:** `turn = -(kp·offset + ka·angle)`, forward `max(min_drive, speed·(1 - |turn|/2))`,
and above `|turn|` 0.7 it turns in place. Fractions of `command_node`'s `max_speed`; robot01's
mecanum wheels don't move below ~0.6, so `min_drive` is 0.6 and the default speed 0.7 (it was
0.3, and the dashboard's 0.25 would not have moved the robot even with the lane found). It stops
on a `stop` command (dashboard and voice STOP), when the camera goes, or when a search finds
nothing; `command_node` still stops the robot 0.5 s after the last `cmd_vel` and at an obstacle.
`vision/state` shows `phase`: idle, search, turn, follow or failed.

## YOLOX

COCO's 80 classes. Detection runs on its own thread with 3 ncnn threads (leaving a core for
the camera, lane tracking and ROS), as fast as it can; it is off by default because it keeps
three cores busy. Measured on robot01 (Pi 4, 4 GB): lane tracking 2.8 ms/frame; YOLOX nano 124 ms (8.1 frames/s)
and tiny 274 ms (3.7 frames/s) with 3 threads, about 150 ms for nano while the node also tracks
the lane. Measure on your Pi:

```bash
~/ros2_ws/install/robot_vision/lib/robot_vision/vision_bench
```

Model files: `/usr/local/share/robot_vision/models/yolox{N,T}.{param,bin}`, from
[Qengineering/YoloX-ncnn-Raspberry-Pi-4](https://github.com/Qengineering/YoloX-ncnn-Raspberry-Pi-4)
(BSD-3-Clause; YOLOX weights by Megvii, Apache-2.0).
