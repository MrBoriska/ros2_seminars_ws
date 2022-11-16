echo "[Set the ROS evironment]"


name_ros_version=${name_ros_version:="humble"}
name_colcon_workspace=${name_colcon_workspace:="ros2_seminars_ws"}
ros_domain_id=$(ip -o addr show dev "enp0s31f6" | awk '$3 == "inet" {print $4}' | sed -r 's!/.*!!; s!.*\.!!')
student_home=/home/student

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source $student_home/$name_colcon_workspace/install/local_setup.bash\" >> $student_home/.bashrc"
sh -c "echo \"export ROS_DOMAIN_ID=$ros_domain_id\" >> $student_home/.bashrc"
