#!/bin/bash

echo ""
echo "[Note] OS version  >>> Ubuntu 22.04"
echo "[Note] Target ROS version >>> ROS 2 Humble"
echo "[Note] Colcon workspace   >>> $HOME/ros2_seminars_ws"
echo ""

echo "[Set the target ROS version and name of colcon workspace]"
name_ros_version=${name_ros_version:="humble"}
name_colcon_workspace=${name_colcon_workspace:="ros2_seminars_ws"}
student_home=$HOME

sudo apt update
sudo apt install -y ros-$name_ros_version-gazebo-ros-pkgs \
                    ros-$name_ros_version-joint-state-publisher \
                    ros-$name_ros_version-xacro \
                    ros-$name_ros_version-rqt-robot-steering \
                    ros-$name_ros_version-cartographer \
                    ros-$name_ros_version-cartographer-ros \
                    ros-$name_ros_version-navigation2 \
                    ros-$name_ros_version-nav2-bringup \
                    ros-$name_ros_version-dynamixel-sdk \
                    ros-$name_ros_version-turtlebot3-msgs \
                    ros-$name_ros_version-turtlebot3 \
                    ros-$name_ros_version-turtlebot3-simulations \
                    ros-$name_ros_version-ament-cmake \
                    ros-$name_ros_version-ament-cmake* \
                    ros-$name_ros_version-robot-state-publisher \
                    ros-$name_ros_version-robot-state-publisher-gui
                    

echo "[Environment setup]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool git

#sudo rosdep init
#rosdep update

echo "[Make the colcon workspace and test colcon build]"
#mkdir -p $HOME/$name_colcon_workspace/src
cd $student_home/$name_colcon_workspace
#rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install

echo "[Complete!!!]"
exit 0
