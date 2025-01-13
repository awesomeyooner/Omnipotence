from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch
import launch_ros

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(get_package_share_directory('camera_driver'),'config','camera_driver_config.yaml')

    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='camera_driver',
            executable='camera_driver_node',
            name='camera_driver_node',
            parameters=[params]
            )

            #('/cmd_vel', '/diffbot_base_controller/cmd_vel')
  ])