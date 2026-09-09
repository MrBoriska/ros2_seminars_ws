#!/bin/bash
set -e

echo ""
echo "=========================================================="
echo "[Note] OS version          >>> Ubuntu 22.04 LTS"
echo "[Note] Target ROS version  >>> ROS 2 Humble Hawksbill"
echo "[Note] Workspace Location  >>> $(cd "$(dirname "$0")/.." && pwd)"
echo "=========================================================="
echo ""

name_ros_version=${name_ros_version:="humble"}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "[1/4] Updating package lists..."
sudo apt update

echo "[2/4] Installing ROS 2 dependencies and Gazebo Classic..."
sudo apt install -y \
    ros-$name_ros_version-gazebo-ros-pkgs \
    ros-$name_ros_version-joint-state-publisher \
    ros-$name_ros_version-joint-state-publisher-gui \
    ros-$name_ros_version-robot-state-publisher \
    ros-$name_ros_version-xacro \
    ros-$name_ros_version-rqt-robot-steering \
    ros-$name_ros_version-rqt-graph \
    ros-$name_ros_version-teleop-twist-keyboard \
    ros-$name_ros_version-cartographer \
    ros-$name_ros_version-cartographer-ros \
    ros-$name_ros_version-navigation2 \
    ros-$name_ros_version-nav2-bringup \
    ros-$name_ros_version-dynamixel-sdk \
    ros-$name_ros_version-turtlebot3-msgs \
    ros-$name_ros_version-turtlebot3 \
    ros-$name_ros_version-turtlebot3-simulations \
    ros-$name_ros_version-ament-cmake \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pip \
    git

echo "[3/4] Sourcing ROS 2 environment..."
if [ -f "/opt/ros/$name_ros_version/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "/opt/ros/$name_ros_version/setup.bash"
else
    echo "Warning: /opt/ros/$name_ros_version/setup.bash not found. Please install ros-$name_ros_version-desktop."
fi

echo "[4/4] Building colcon workspace at $WORKSPACE_DIR..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install

echo ""
echo "[Complete! Dependencies installed and workspace built successfully.]"
exit 0
