#!/usr/bin/env bash
set -e

#source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/$WS_NAME
rosdep update
rosdep install -yrq --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build