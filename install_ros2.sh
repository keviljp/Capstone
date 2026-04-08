#!/bin/bash
# ROS2 Jazzy + swarm_state_estimation dependency installer
# Run from a terminal: bash install_ros2.sh
set -e

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== [1/6] Locale setup ==="
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== [2/6] Adding ROS2 Jazzy apt repository ==="
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor \
    | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "=== [3/6] Installing ROS2 Jazzy + required packages ==="
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-cv-bridge \
    ros-jazzy-apriltag-ros \
    ros-jazzy-apriltag-msgs \
    ros-jazzy-robot-localization \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    v4l-utils

echo "=== [4/6] Initializing rosdep ==="
sudo rosdep init 2>/dev/null || echo "(rosdep already initialized, skipping)"
rosdep update

echo "=== [5/6] Adding ROS2 source to ~/.bashrc ==="
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "Added ROS2 source to ~/.bashrc"
else
    echo "Already in ~/.bashrc, skipping"
fi

echo "=== [6/6] Building the workspace ==="
source /opt/ros/jazzy/setup.bash
cd "$WORKSPACE_DIR"
colcon build --symlink-install

if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
fi

echo ""
echo "=========================================="
echo " Done! Run the following to finish setup:"
echo "   source ~/.bashrc"
echo ""
echo " Then launch the full stack with:"
echo "   ros2 launch swarm_state_estimation live_demo.launch.py"
echo ""
echo " Or camera-only:"
echo "   ros2 launch swarm_state_estimation camera_only.launch.py"
echo "=========================================="
