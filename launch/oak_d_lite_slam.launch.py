import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Get the rtabmap_ros share directory for default RViz config
    rtabmap_ros_share_dir = get_package_share_directory('rtabmap_ros')

    # Parameters for RTAB-Map
    parameters = {
        'frame_id': 'oak_d_camera_base_link',  # A common frame ID for OAK-D cameras. Adjust if your depthai-ros driver uses a different one (e.g., 'oak_d_link' or 'base_link')
        'subscribe_depth': True,               # We have /stereo/depth, so subscribe to it
        'subscribe_rgbd': False,               # Not subscribing to a combined RGBD topic
        'subscribe_stereo': False,             # Don't subscribe to raw stereo images for depth generation, as /stereo/depth is already provided
        'approx_sync': True,                   # Set to True if topics are not perfectly synchronized
        'wait_for_transform': 0.1,             # Time to wait for transforms between frames
        'odom_frame_id': 'odom',               # Output odometry frame
        'map_frame_id': 'map',                 # Output map frame
        'publish_tf': True,                    # Publish TF transforms
        'database_path': os.path.expanduser('~/.ros/rtabmap.db'), # Path for the database
        'rtabmap_args': '--delete_db_on_start', # Add this to start with a fresh map each time. Remove for persistent mapping.
        
        # Odometry strategy for visual/depth only (no IMU)
        'Odom/Strategy': '0',                  # 0=Frame-to-Map (Bundle Adjustment), 1=Frame-to-Frame, 2=Bundle Adjustment, 9=IMU-based (DISABLED HERE)
        'Odom/Force2D': 'false',                # Set to true if you are only moving on a flat plane
        'Reg/ForceICP': 'false',               # Force ICP for registration (good for unstructured environments)
        'Vis/MaxFeatures': '1000',             # Max features to extract for visual odometry
        'Vis/MinInliers': '15',                # Min inliers for visual odometry to succeed
        'Mem/ReactivatedNodeActivated': 'true',# Enable reactivation of old nodes for loop closure
    }

    # Input topics from OAK-D Lite ROS 2 driver (based on your list)
    remappings = [
        ('rgb/image', '/color/video/image'),
        ('rgb/camera_info', '/color/video/camera_info'),
        ('depth/image', '/stereo/depth'),
        # No stereo image remapping needed as we subscribe to depth directly
        # No IMU remapping needed as we determined no functional IMU
    ]

    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d'] # Delete the database on start, remove for persistent mapping
    )

    rtabmap_odom_node = Node(
        package='rtabmap_ros',
        executable='rgbd_odometry', # or stereo_odometry if you prefer
        output='screen',
        parameters=[parameters], # Share parameters with the rtabmap node
        remappings=remappings,
    )

    # RViz2 for visualization
    rviz_config_file = os.path.join(rtabmap_ros_share_dir, 'launch', 'config', 'rgbd.rviz')
    if not os.path.exists(rviz_config_file):
        print(f"WARNING: RViz config file not found at {rviz_config_file}. RViz might not start correctly.")
        # Fallback to a generic RViz if the specific config isn't found
        rviz_config_file = '' 

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
    )

    return [
        rtabmap_node,
        rtabmap_odom_node,
        rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_image_topic',
            default_value='/color/video/image',
            description='Topic for RGB image'
        ),
        DeclareLaunchArgument(
            'rgb_camera_info_topic',
            default_value='/color/video/camera_info',
            description='Topic for RGB camera info'
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/stereo/depth',
            description='Topic for Depth image'
        ),
        # No IMU argument needed as we are not using it
        OpaqueFunction(function=launch_setup)
    ])