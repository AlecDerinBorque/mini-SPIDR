# mini-SPIDR

A simulated pan/tilt camera tracking pipeline built with ROS 2 (Jazzy). A wide video stands in for the world, a virtual camera sees a cropped window of it, a YOLOv8n detector finds a target in that window, and a PID controller drives a simulated pan/tilt unit (PTU) to keep the target centered. Moving the PTU moves the crop window, so the loop closes entirely in software.

Live demo: https://youtu.be/p5B0g-zbFqU?si=xdK3A30o0WNsZE28

## How it works

```
                 ptu/state (pan, tilt deg)
        +-------------------------------------------+
        v                                           |
  frame_node --camera/image_raw--> detector_node    |
  (wide video,                     (YOLOv8n,        |
   crop by pan/tilt)               nearest-target   |
                                   tracking)        |
                                        |           |
                              detector/target       |
                                        v           |
                                 controller_node    |
                                 (PID, px -> deg)   |
                                        |           |
                                cmd/auto (deg/s)    |
                                        v           |
                                  ptu_sim_node -----+
                                  (rate -> angle,
                                   accel + limits)
```

| Node | Subscribes | Publishes | What it does |
|---|---|---|---|
| `frame_node` | `ptu/state` | `camera/image_raw` | Plays a wide video on loop and publishes only the 640x360 crop the camera would see at the current pan/tilt. |
| `detector_node` | `camera/image_raw` | `detector/target`, `detector/debug_image` | Runs YOLOv8n on CPU and publishes at most one `vision_msgs/Detection2D`. Sticks to the detection nearest the previous one (within a pixel gate), otherwise picks the most confident. |
| `controller_node` | `detector/target` | `cmd/auto`, `controller/error_deg` | Converts the target's pixel offset from image center into degrees, then runs one PID per axis to produce a rate command in deg/s. Gains can be changed at runtime. |
| `ptu_sim_node` | `ptu/cmd_rate` | `ptu/state` | Integrates rate commands into pan/tilt angles with a max rate, max acceleration, travel limits and a command timeout. |

Conventions: `ptu/state` and the commands are `geometry_msgs/Vector3Stamped` with `x` = pan in degrees (positive right) and `y` = tilt in degrees (positive up). The camera model is `px_per_deg = 20`, so a 1280x720 video with a 640x360 crop gives a pan range of +/-16 deg and a tilt range of +/-9 deg.

## Requirements

- ROS 2 Jazzy with `rclpy`, `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `vision_msgs`
- Python packages: `ultralytics` and `numpy<2` (see [config/constraints.txt](config/constraints.txt)). ROS's OpenCV is built against NumPy 1.x, so a NumPy 2 install breaks `import cv2`.
- A wide video (for example 1280x720) containing the thing you want to track. Videos are gitignored, so bring your own.
- A YOLOv8n weights file, or pass `model_path:=yolov8n.pt` and let Ultralytics download it.

## Build

```bash
cd ros2_ws
colcon build --packages-select mini_spidr
source install/setup.bash
```

## Fast DDS shared-memory profile

A 640x360 BGR frame is about 691 KB, which is larger than Fast DDS's default shared-memory segment, so image transport silently falls back to fragmented UDP and the frame rate collapses. [config/fastdds_shm.xml](config/fastdds_shm.xml) enlarges the segment to 32 MB. Set it in every terminal before running the nodes:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/config/fastdds_shm.xml
```

## Run

The controller publishes `cmd/auto` and the PTU listens on `ptu/cmd_rate`, so remap one onto the other. The controller's gains default to zero (no motion), so set them.

Run each in its own terminal:

```bash
ros2 run mini_spidr frame_node --ros-args -p video_path:=/path/to/wide.mp4

ros2 run mini_spidr detector_node --ros-args -p model_path:=/path/to/yolov8n.pt

ros2 run mini_spidr controller_node --ros-args \
  -p kp:=1.0 -p ki:=0.0 -p kd:=0.0 \
  -r cmd/auto:=ptu/cmd_rate

ros2 run mini_spidr ptu_sim_node
```

Watch the result:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `detector/debug_image`: the green box is the tracked target, grey boxes are other candidates, and the cross marks the image center. `controller/error_deg` is the pan/tilt error the PID sees.

Retune live:

```bash
ros2 param set /controller_node kp 2.0
```

## Parameters

| Node | Parameter | Default | Meaning |
|---|---|---|---|
| `frame_node` | `video_path` | `/home/ubuntu/mini-SPIDR/data/wide.mp4` | Source video |
| | `crop_width`, `crop_height` | 640, 360 | Camera field of view in pixels |
| | `fps` | 15.0 | Publish rate |
| | `px_per_deg` | 20.0 | Pixels per degree of pan/tilt |
| `detector_node` | `model_path` | `/home/ubuntu/mini-SPIDR/models/yolov8n.pt` | YOLO weights |
| | `target_class` | `person` | COCO class to track |
| | `conf` | 0.25 | Detection confidence threshold |
| | `imgsz` | 640 | Inference size |
| | `gate_px` | 150.0 | Max jump to keep the same target |
| | `forget_after` | 10 | Missed frames before dropping the target |
| `controller_node` | `kp`, `ki`, `kd` | 0.0 | PID gains (deg/s per deg of error) |
| | `out_limit` | 40.0 | Rate command limit, deg/s |
| | `i_limit` | 10.0 | Integral windup clamp |
| | `image_width`, `image_height` | 640, 360 | Must match the crop size |
| | `px_per_deg` | 20.0 | Must match `frame_node` |
| `ptu_sim_node` | `max_rate` | 40.0 | deg/s |
| | `max_accel` | 300.0 | deg/s^2 |
| | `pan_min`, `pan_max` | -16.0, 16.0 | Pan travel, deg |
| | `tilt_min`, `tilt_max` | -9.0, 9.0 | Tilt travel, deg |
| | `update_hz` | 50.0 | Simulation rate |
| | `cmd_timeout` | 0.5 | Seconds without a command before stopping |

## Limitations

- No launch file yet; nodes are started by hand as above.
- Detection runs on CPU at roughly 80-130 ms per frame, so the loop effectively runs at ~8-12 Hz.
- YOLOv8n struggles with small targets (tens of pixels tall). Use footage where the target is large in frame.
- The two axes share one set of PID gains.

## License

Declared as MIT in `package.xml`.
