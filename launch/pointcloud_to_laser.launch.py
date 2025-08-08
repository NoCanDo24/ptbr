import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    point_cloud_creator = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyziNode',
        name='point_cloud_xyzi',
        remappings=[('depth/image_rect', 'stereo/converted_depth'),
                    ('intensity/image_rect', 'color/image'),
                    ('intensity/camera_info', 'color/camera_info'),
                    ('points', 'laser/points')]
    )
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                "target_frame": "base_link",
                "min_height": 0.05,
                "max_height": 0.3,
                "angle_increment": 0.0043633231,
                "range_max": 3.0, 
                "use_sim_time": False,
                "use_sensor_data_qos": True
            }],
            remappings=[
                ("cloud_in", "laser/points"),
                ("scan", "/scan")
            ],
        ),

        ComposableNodeContainer(
                name='container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    # Driver itself
                    point_cloud_creator
                ],
                output='screen',)
    ])