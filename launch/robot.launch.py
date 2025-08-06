import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('ptbr')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Path to your parameter file
    params_file = os.path.join(pkg_share_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # # Your robot description launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch', 'rsp.launch.py'))
        # ),
        
        # # RTAB-Map launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch', 'slam.launch.py'))
        # ),
        
        # Your custom motor controller node
        # Node(
        #     package='ptbr',
        #     executable='motor_controller',
        #     name='motor_controller',
        #     output='screen'
        # ),
        
        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'params_file': params_file,
                'autostart': 'true',
                'use_sim_time': 'false',
            }.items()
        )
    ])