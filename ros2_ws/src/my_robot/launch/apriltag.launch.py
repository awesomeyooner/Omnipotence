import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    camera_driver_package = "camera_driver"

    camera_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(camera_driver_package), "launch", "camera_driver.launch.py"))
    )

    image_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(camera_driver_package), "launch", "image_proc.launch.py"))
    )

    apriltag_package = "apriltag_ros"

    apriltag_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(apriltag_package), "launch", "detector.launch.py"))
    )

    apriltag_translator_package = "apriltag_translator"

    apriltag_translator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(apriltag_translator_package), "launch", "apriltag_translator.launch.py"))
    )

    return LaunchDescription([
        camera_driver_launch,
        image_proc_launch,
        apriltag_detector_launch,
        apriltag_translator_launch
    ])