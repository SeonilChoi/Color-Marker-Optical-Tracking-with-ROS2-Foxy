import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("camera_pkg"),
            'params',
            'camera_parameters_config.yaml')
    )

    left_image_publisher = Node(
        package="camera_pkg",
        executable="image_publisher",
        name="image_publisher",
        namespace="left",
        parameters=[param_dir],
        output="screen"
    )
    
    right_image_publisher = Node(
        package="camera_pkg",
        executable="image_publisher",
        name="image_publisher",
        namespace="right",
        parameters=[param_dir],
        output="screen"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "param_dir",
            default_value = param_dir,
            description="Full path of parameter file."
        ),
        left_image_publisher,
        right_image_publisher,
    ])
    
