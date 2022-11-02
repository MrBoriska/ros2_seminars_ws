FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND noninteractive
ENV DEV_NAME=user
ENV ROS_DISTRO=humble
ENV GROUP_NAME=ros
ENV WS_NAME=ros2_seminars_ws


RUN useradd --create-home --home-dir /home/${DEV_NAME} --shell /bin/bash --user-group --groups adm,sudo ${DEV_NAME} && \
    echo "$DEV_NAME:$DEV_NAME" | chpasswd && \
    echo "$DEV_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
COPY --chown=${DEV_NAME} . /home/${DEV_NAME}/${WS_NAME}

COPY ros2-setup.bash /bin/ros2-setup.bash

# install unityhub
#RUN apt-get -y wget && \
#    sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list' && \
#    wget -qO - https://hub.unity3d.com/linux/keys/public | sudo apt-key add - && \
#    apt-get update && \
#    apt-get install unityhub

# Doing a second fetch of sources & apt-get update here, because these ones depend on the state of the build context
# in our repo
RUN apt-get update -q && \
    chmod +x /bin/ros2-setup.bash && \
    runuser -u ${DEV_NAME} ros2-setup.bash && \
    rm /bin/ros2-setup.bash

RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${DEV_NAME}/.bashrc && \
    echo ". /home/${DEV_NAME}/${WS_NAME}/install/local_setup.bash" >> /home/${DEV_NAME}/.bashrc


WORKDIR /home/${DEV_NAME}/${WS_NAME}

ENV TURTLEBOT3_MODEL=waffle_pi
# To bring up tb3 simulation example (from https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)
# cd catkin_ws && source install/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True

# Informs the environment that the default user is not root, but instead DEV_NAME
ENV USER ${DEV_NAME}