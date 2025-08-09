import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('ptbr')

    return LaunchDescription([
        # Joint launch file (also launches motors)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_share_dir, 'launch', 'joints.launch.py')])
        ),

        # Slam launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_share_dir, 'launch', 'slam.launch.py')])
        ),
        

    ])