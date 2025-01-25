# Author: Addison Sears-Collins
# Date: August 31, 2021
# Description: Launch a basic mobile robot
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/

def generate_launch_description():

  # Set the path to different files and folders.
  pkg_share = FindPackageShare(package='pose_estimator').find('pose_estimator')
  robot_localization_file_path = os.path.join(pkg_share, 'config', 'ekf.yaml') 

  use_sim_time = LaunchConfiguration('use_sim_time')

  # Declare the launch arguments  

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
  # Specify the actions

  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_use_sim_time_cmd)

  # Add any actions
  ld.add_action(start_robot_localization_cmd)

  return ld