from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   # Paths
   sllidar_launch_path = os.path.join(
       get_package_share_directory('sllidar_ros2'),
       'launch',
       'sllidar_a3_launch.py'
   )

   # Robot Control Group
   robot_control_group = GroupAction([
       Node(
           package='robot_control',
           executable='lidar_data_converter',
           remappings=[
                ('/scan', '/scan/turtlebot3') #remap /scan to /scan/turtlebot3
            ]
       ),
       Node(
           package='robot_control',
           executable='motion_controller'
       ),
       Node(
           package='robot_control',
           executable='obstacle_detector'
       )
   ])

   # Lidar Group with SetRemap
   lidar_group = GroupAction(
        actions=[
            SetRemap(src='/scan', dst='/scan/a3'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sllidar_launch_path)
            )
        ]
   )

   return LaunchDescription([
       robot_control_group,
       lidar_group
   ])