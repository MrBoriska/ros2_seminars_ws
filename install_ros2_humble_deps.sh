#!/bin/bash

echo ""
echo "[Note] OS version  >>> Ubuntu 22.04"
echo "[Note] Target ROS version >>> ROS 2 Humble"
echo "[Note] Colcon workspace   >>> $HOME/ros2_seminars_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target ROS version and name of colcon workspace]"
name_ros_version=${name_ros_version:="humble"}
name_colcon_workspace=${name_colcon_workspace:="ros2_seminars_ws"}
ros_domain_id=1 # change this for all computers

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
                    ros-$name_ros_version-ament-cmake*
                    

echo "[Environment setup]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool python-rosdep2 git

sudo rosdep init
rosdep update

echo "[Make the colcon workspace and test colcon build]"
#mkdir -p $HOME/$name_colcon_workspace/src
cd $HOME/$name_colcon_workspace
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install

if [ -z "$1" ]; then
    echo "[Set the ROS evironment]"

    sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
    sh -c "echo \"source ~/$name_colcon_workspace/install/local_setup.bash\" >> ~/.bashrc"
    sh -c "echo \"export ROS_DOMAIN_ID=$ros_domain_id\" >> ~/.bashrc"
fi

source $HOME/.bashrc

echo "[Complete!!!]"
exit 0
