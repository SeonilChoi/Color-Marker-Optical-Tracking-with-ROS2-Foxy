import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("optical_marker_tracking"),
            'params',
            'optical_tracking_parameters_config.yaml')
    )

    left_marker_detector = Node(
        package="optical_marker_tracking",
        executable="marker_detector",
        name="marker_detector",
        namespace="left",
        output="screen"
    )
    
    right_marker_detector = Node(
        package="optical_marker_tracking",
        executable="marker_detector",
        name="marker_detector",
        namespace="right",
        output="screen"
    )
    
    marker_matcher = Node(
        package="optical_marker_tracking",
        executable="marker_matcher",
        name="marker_matcher",
        parameters=[param_dir],
        output="screen"
    )
    
    marker_triangulator = Node(
        package="optical_marker_tracking",
        executable="marker_triangulator",
        name="marker_triangulator",
        parameters=[param_dir],
        output="screen"
    )
    
    marker_tracker = Node(
        package="optical_marker_tracking",
        executable="marker_tracker",
        name="marker_tracker",
        parameters=[param_dir],
        output="screen"
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "param_dir",
            default_value = param_dir,
            description="Full path of parameter file."
        ),
        left_marker_detector,
        right_marker_detector,
        marker_matcher,
        marker_triangulator,
        marker_tracker,
    ])
    
    
