#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/humble/setup.bash
fi

sudo apt update
sudo apt install -y   python3-colcon-common-extensions   python3-vcstool   python3-rosdep   python3-opencv   python3-yaml   ros-humble-cv-bridge   ros-humble-robot-localization   ros-humble-camera-calibration   ros-humble-rviz2   ros-humble-tf2-ros   ros-humble-tf-transformations

if [[ ! -d src/apriltag_ros ]]; then
  vcs import src < swarm_state_humble.repos
fi

rosdep update || true
rosdep install --from-paths src --ignore-src -r -y

echo "Bootstrap complete."
echo "Next steps:"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "  ros2 launch swarm_state_estimation live_demo.launch.py"
