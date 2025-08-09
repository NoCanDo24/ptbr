import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='ptbr',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='ptbr',
            executable='joint_controller',
            name='joint_controller',
            output='screen'
        ),
        Node(
            package='ptbr',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])