# Author: Addison Sears-Collins
# Date: August 27, 2021
# Description: Launch a basic mobile robot URDF file using Rviz.
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
#from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions.execute_process import ExecuteProcess

import xacro

def generate_launch_description():
  pkg_install_dir = get_package_prefix('basic_mobile_robot')

  if 'GAZEBO_MODEL_PATH' in os.environ:
      os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + pkg_install_dir + '/share'
  else:
      os.environ['GAZEBO_MODEL_PATH'] =  pkg_install_dir + "/share"

  if 'GAZEBO_PLUGIN_PATH' in os.environ:
      os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + pkg_install_dir + '/lib'
  else:
      os.environ['GAZEBO_PLUGIN_PATH'] = pkg_install_dir + '/lib'

  try:
    envs = {}
    for key in os.environ.__dict__["_data"]:
      key = key.decode("utf-8")
      if (key.isupper()):
        envs[key] = os.environ[key]
  except Exception as e:
    print("Error with Envs: " + str(e))
    return None

  # Set the path to different files and folders.
  pkg_share = get_package_share_directory('basic_mobile_robot')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_model_path = os.path.join(pkg_share, 'models/basic_mobile_bot.xacro.urdf')
  default_model_path_parsed = os.path.join(pkg_share, 'models/basic_mobile_bot_v1.urdf')
  robot_name_in_urdf = 'basic_mobile_bot'
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
  
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  model = LaunchConfiguration('model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')

  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  """
  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
  """
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
   
  # Specify the actions

  """
  # Publish the joint state values for the non-fixed joints in the URDF file.
  start_joint_state_publisher_cmd = Node(
    condition=UnlessCondition(gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher')

  # A GUI to manipulate the joint state values
  start_joint_state_publisher_gui_node = Node(
    condition=IfCondition(gui),
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui')
  """

  # Parse xacro to urdf
  rdc = xacro.process_file(default_model_path)
  robot_description = rdc.toxml()
  f = open(default_model_path_parsed, 'w')
  f.write(robot_description)
  f.close()

  #print(robot_description)

  # GAZEBO EXECUTION
  start_gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    output='screen',
    env=envs
  )

  # ROBOT MODEL GAZEBO SPAWNER
  spawn_entity = ExecuteProcess(
    cmd=['python3', 'spawn_gazebo.py', default_model_path_parsed, 'basic_mobile_bot'],
    output='screen',
    cwd=default_launch_dir
  )

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
      'use_sim_time': use_sim_time#, 
      #'robot_description': robot_description
    }],
    arguments=[default_model_path_parsed]
  )
  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  #ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)

  # Add any actions
  ld.add_action(start_gazebo)
  #ld.add_action(start_joint_state_publisher_cmd)
  #ld.add_action(start_joint_state_publisher_gui_node)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(spawn_entity)

  return ld
