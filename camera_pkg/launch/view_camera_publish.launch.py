import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    this_pkg_path = os.path.join(get_package_share_directory('camera_pkg'))
    world_file_path = os.path.join(this_pkg_path, 'urdf', 'table.urdf.xacro')
    rviz_path = os.path.join(this_pkg_path, 'rviz', 'camera_geometry.rviz') 
        
    with open(world_file_path, 'r') as f:
        doc = xacro.parse(f)
        xacro.process_doc(doc)
        world_description = doc.toxml()
    
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
    
    static_camera_tf2_broadcaster = Node(
        package="camera_pkg",
        executable="static_camera_tf2_broadcaster",
        name="static_camera_tf2_broadcaster",
        parameters=[param_dir],
        output="screen"
    )
    
    static_world_tf2_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['-0.66', '-0.24', '0.0', '1.570796327', '0', '3.141592654', 'world', 'table']
    )
    
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters = [{
            'robot_description':world_description
        }],
        arguments=[world_file_path],
        namespace="world"
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path],
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
        static_camera_tf2_broadcaster,
        static_world_tf2_broadcaster,
        robot_state_publisher,
        rviz2
    ])
    
