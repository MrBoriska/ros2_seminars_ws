#!/bin/bash
# Apache License 2.0
# Copyright (c) 2019, ROBOTIS CO., LTD.

echo ""
echo "[Note] OS version  >>> Ubuntu 18.04 (Bionic Beaver) or Linux Mint 19.x"
echo "[Note] Target ROS version >>> ROS 2 Dashing Diademata"
echo "[Note] Colcon workspace   >>> $HOME/ros2_seminars_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target ROS version and name of colcon workspace]"
name_ros_version=${name_ros_version:="dashing"}
name_colcon_workspace=${name_colcon_workspace:="ros2_seminars_ws"}

echo "[Setup Locale]"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Setup Sources]"
sudo rm -rf /var/lib/apt/lists/* && sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

echo "[Install ROS 2 packages]"
sudo apt update
sudo apt install -y ros-$name_ros_version-desktop
sudo apt remove -y gazebo11 libgazebo11-dev
sudo apt install -y gazebo9 libgazebo9-dev
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
                    ros-$name_ros_version-ament-cmake*
                    

echo "[Environment setup]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool python-rosdep2 git

sudo rosdep init
rosdep update

echo "[Make the colcon workspace and test colcon build]"
#mkdir -p $HOME/$name_colcon_workspace/src
cd $HOME/$name_colcon_workspace
rosdep install -i --from-path src --rosdistro dashing -y
colcon build --symlink-install

if [ -z "$1" ]; then
    echo "[Set the ROS evironment]"
    sh -c "echo \"alias nb='nano ~/.bashrc'\" >> ~/.bashrc"
    sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
    sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
    sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"

    sh -c "echo \"alias cw='cd ~/$name_colcon_workspace'\" >> ~/.bashrc"
    sh -c "echo \"alias cs='cd ~/$name_colcon_workspace/src'\" >> ~/.bashrc"
    sh -c "echo \"alias cb='cd ~/$name_colcon_workspace && colcon build --symlink-install && source ~/.bashrc'\" >> ~/.bashrc"

    sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
    sh -c "echo \"source ~/$name_colcon_workspace/install/local_setup.bash\" >> ~/.bashrc"
fi

source $HOME/.bashrc

echo "[Complete!!!]"
exit 0
