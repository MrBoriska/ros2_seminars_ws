FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install GUI utilities, tools and seminar dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    wget \
    curl \
    nano \
    vim \
    bash-completion \
    python3-pip \
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-vcstool \
    # ROS 2 CLI & State Publisher Tools
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    # RQt & Visualization Tools
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rviz2 \
    # Teleoperation & Simulation
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # Modern Gazebo Fortress integration in Humble
    ros-${ROS_DISTRO}-ros-gz \
    # Robot Models & Tutorials
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Create non-root student user
ARG USERNAME=student
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME 2>/dev/null || true \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup automatic environment sourcing for interactive shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /workspaces/ros2_seminars_ws/install/setup.bash ]; then source /workspaces/ros2_seminars_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && echo "export TURTLEBOT3_MODEL=burger" >> /home/${USERNAME}/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models:/workspaces/ros2_seminars_ws/src/youbot_description/sdf" >> /home/${USERNAME}/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/${USERNAME}/.bashrc

USER $USERNAME
WORKDIR /workspaces/ros2_seminars_ws

CMD ["/bin/bash"]
