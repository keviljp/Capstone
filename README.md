# swarm_state_ws

ROS 2 Humble workspace for Josh's capstone state-estimation subsystem:

- USB overhead camera on the host computer
- AprilTag detection using `apriltag_ros`
- Tag-ID-to-robot-ID routing
- Dynamic UKF startup per robot after first sighting
- Filtered team-facing output on `/swarmbot_<id>/odom`

## What this repo does

This repo owns **your part only**:

- reads a USB camera
- publishes `/camera/image_raw`, `/camera/image_rect`, and `/camera/camera_info`
- runs `apriltag_ros`
- converts detections into per-robot absolute pose measurements
- starts one UKF per seen robot
- outputs `/swarmbot_<id>/odom`

This repo does **not** include ESP32 firmware or teammate planning/control code. It only expects that each robot's IMU arrives on:

- `/swarmbot_<id>/imu/data` (`sensor_msgs/Imu`)

## Repository layout

```text
swarm_state_ws/
├── README.md
├── bootstrap.sh
├── swarm_state_humble.repos
└── src/
    └── swarm_state_estimation/
        ├── config/
        ├── launch/
        ├── resource/
        ├── rviz/
        ├── scripts/
        ├── swarm_state_estimation/
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## Install

```bash
cd swarm_state_ws
chmod +x bootstrap.sh
./bootstrap.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Required edits before first real run

### 1) Put your real camera calibration here
Replace:

```text
src/swarm_state_estimation/config/camera_info/overhead_camera.yaml
```

with your real calibration.

### 2) Fill in your real tag-to-robot map
Edit:

```text
src/swarm_state_estimation/config/tag_map.yaml
```

Example:

```yaml
tag_to_robot_id:
  0: "7c60"
  1: "1f2a"
  2: "9b31"
```

### 3) Confirm your tag size
Default is set to `0.050` m in:

```text
src/swarm_state_estimation/config/apriltag_36h11.yaml
```

Change it to your real printed tag size.

## Run

```bash
source /opt/ros/humble/setup.bash
cd swarm_state_ws
source install/setup.bash
ros2 launch swarm_state_estimation live_demo.launch.py
```

Optional:

```bash
ros2 launch swarm_state_estimation live_demo.launch.py use_rviz:=true
```

## Expected live topics

### Camera side
- `/camera/image_raw`
- `/camera/image_rect`
- `/camera/camera_info`
- `/detections`

### Your internal localization topics
- `/swarmbot_<id>/tag_pose`
- `/swarm_state/discovered_robot_id`

### Team-facing output
- `/swarmbot_<id>/odom`

### Required teammate input
- `/swarmbot_<id>/imu/data`

## Expected topic contract

Each robot's filtered state is published as:

- `/swarmbot_7c60/odom`

and should be consumed by the rest of the stack.

## Micro-ROS note

Your robots may use micro-ROS on the ESP32, but that is **not implemented in this repo**. This repo only assumes that the ROS 2 host can already see IMU topics on `/swarmbot_<id>/imu/data`. If your teammates are using micro-ROS, they will need to run a micro-ROS Agent on the ROS 2 side so the MCU topics appear in the ROS graph.

## Design choices in this repo

### Why `36h11`
You asked why I picked `36h11` instead of `25h9` or `16h5`.

`apriltag_ros` supports multiple families including `16h5`, `25h9`, and `36h11`. I set the default to `36h11` because it is a common, well-supported default with more coding space and stronger coding distance. For fewer than 10 robots, `25h9` would also be perfectly reasonable. `16h5` is the one I would be least likely to choose unless you specifically need the smallest family. If you want, you can switch families by changing one line in `config/apriltag_36h11.yaml`.

### Why filters start only after first sighting
You wanted one UKF per robot that starts once the robot is first seen, and keeps propagating on IMU after it leaves view. That is exactly what `ukf_manager.py` does.

### Why no teammate messages package
You said your part is self-contained and should only own:
- camera input
- IMU input
- pose/yaw output

So I did **not** add docking or planning custom messages here.

## Quick validation commands

See active topics:

```bash
ros2 topic list
```

Watch detections:

```bash
ros2 topic echo /detections
```

Watch one robot output:

```bash
ros2 topic echo /swarmbot_7c60/odom
```

See which UKFs were started:

```bash
ros2 topic echo /swarm_state/discovered_robot_id
```

## Calibration helper

Install the ROS camera calibration package if needed:

```bash
sudo apt install ros-humble-camera-calibration
```

A common workflow is:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/camera/image_raw camera:=/camera
```

Then place the resulting YAML into:

```text
src/swarm_state_estimation/config/camera_info/overhead_camera.yaml
```

Adjust `--size` and `--square` to match your checkerboard.
