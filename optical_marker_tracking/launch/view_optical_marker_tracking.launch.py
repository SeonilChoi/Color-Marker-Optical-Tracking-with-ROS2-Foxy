import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    this_pkg_path = os.path.join(get_package_share_directory('optical_marker_tracking'))
    urdf_file_path   = os.path.join(this_pkg_path, 'urdf', 'paddle.urdf.xacro')
    
    with open(urdf_file_path, 'r') as f:
        doc = xacro.parse(f)
        xacro.process_doc(doc)
        robot_description = doc.toxml()
        
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters = [{
            'robot_description':robot_description
        }],
        arguments=[urdf_file_path],
        namespace="robot"
    )

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
    
    marker_tf2_broadcaster = Node(
        package="optical_marker_tracking",
        executable="marker_tf2_broadcaster",
        name="marker_tf2_broadcaster",
        output="screen"
    )
    
    object_tf2_broadcaster = Node(
        package="optical_marker_tracking",
        executable="object_tf2_broadcaster",
        name="object_tf2_broadcaster",
        output="screen"
    )
    
    marker_tracker = Node(
        package="optical_marker_tracking",
        executable="marker_tracker",
        name="marker_tracker",
        parameters=[param_dir],
        output="screen"
    )
    """
    imu_publisher = Node(
        package = "imu_serial_communication",
        executable = "imu_publisher",
        name = "imu_publisher",
        parameters = [{
            "MPU6050":True,
            "AK8963":False
        }]
    )
    
    camera_imu_kalman_filter_publisher = Node(
        package = "camera_imu_kalman_filter",
        executable = "camera_imu_kalman_filter_publisher",
        name = "camera_imu_kalman_filter_publisher",
        parameters = [{
            "rolling_mean":True,
            "rolling_window_size":5
        }]
    )
    """
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
        marker_tf2_broadcaster,
        object_tf2_broadcaster,
        #imu_publisher,
        #camera_imu_kalman_filter_publisher,
        robot_state_publisher
    ])
    
