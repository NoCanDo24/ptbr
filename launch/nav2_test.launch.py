import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('ptbr')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_launch = PathJoinSubstitution(
        [nav2_bringup_dir, 'launch', 'navigation_launch.py'])
    
    # Path to your parameter file
    nav2_params_file = os.path.join(pkg_share_dir, 'config', 'nav2_params.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', nav2_params_file)
        ]
    )

    return LaunchDescription([
        # Your robot description launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch', 'rsp.launch.py'))
        # ),
        
        # RTAB-Map launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(pkg_share_dir, 'launch', 'slam.launch.py')])
        # ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen'
        # ),
        
        # Your custom motor controller node
        # Node(
        #     package='ptbr',
        #     executable='motor_controller',
        #     name='motor_controller',
        #     output='screen'
        # ),
        
        # Nav2 bringup
        nav2,
    ])