from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a3_launch.py'
    )

    return LaunchDescription([
        # My own nodes
        Node(
            package='robot_control',
            executable='lidar_data_converter'
        ),
        Node(
            package='robot_control',
            executable='motion_controller'
        ),
        Node(
            package='robot_control',
            executable='obstacle_detector'
        ),
        
        # Lidar launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path)
        )
    ])