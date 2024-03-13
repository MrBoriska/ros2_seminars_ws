#!/bin/bash

xhost +local:docker
docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix: -v /home/mrboriska/ros2_seminars_ws:/ros2_seminars_ws -it --name ros2_full  osrf/ros:humble-desktop-full