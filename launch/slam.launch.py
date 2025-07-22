# Requirements:
#   A OAK-D camera
#   Install depthai-ros package (https://github.com/luxonis/depthai-ros)
# Example:
#   $ ros2 launch rtabmap_examples depthai.launch.py camera_model:=OAK-D

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters=[{
         'frame_id':'oak-d-base-frame',
         'subscribe_rgbd':True,
         'subscribe_odom_info':True,
         'approx_sync':False,
         'wait_imu_to_init':True,
         'Reg/ForceOdometryUpdate': 'true',
         'Odom/ResetCountdown': '0',
         'Odom/Strategy': '9',
         'Optimizer/GravitySigma': '0.3',
         'Vis/ForceIMUInitialization': 'true',


         'Vis/FeatureType': '6',
         'Vis/MaxFeatures': '1000',
         'Mem/ReduceGraph': 'true',
         
         'Rtabmap/KeyFrameThr': '0.4',
         
         'QueueSize': '10',
         'Rtabmap/PublishTF': 'true',

         'database_path': '~/.ros/rtabmap.db',
         'rtabmap_args': '--delete_db_on_start',

         # NEW: Grid Map Generation Parameters for Nav2
         'publish_map_tf': 'true', # Ensures map->odom transform is published
         'map_manager/grid/footprint_padding': '0.1', # Padding around robot footprint for unknown areas
         'map_manager/grid/from_projected_map': 'false', # True to use 2D projected map, false for 3D voxel grid
         'map_manager/grid/voxel_size': '0.05', # Resolution of the grid map in meters (e.g., 5cm)
         'map_manager/grid/min_occupied_size': '0.05', # Min size for a voxel to be considered occupied
         'map_manager/grid/max_range': '4.0', # Max range for depth points to be considered in the map
         'map_manager/grid/cloud_voxel_size': '0.05', # Voxel filter size for points before grid creation
         'map_manager/grid/map_filter_nodes': 'true', # Filter out nodes not in active graph
         'map_manager/grid/min_map_size': '1.0', # Minimum size of the map to publish
         'map_manager/grid/map_updated': 'true', # Publish map updates
         'Rtabmap/SavePointCloud': 'true',
	     'Rtabmap/SaveImgRaw': 'false',
	     'Rtabmap/SaveDepthRaw': 'false'
	    }]

    remappings=[('imu', '/imu/data')]

    return LaunchDescription([

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py']),
                launch_arguments={'depth_aligned': 'false',
                                  'enableRviz': 'false',
                                  'monoResolution': '400p',
                                  'enable_imu': 'true', # Added: explicitly disable IMU in the camera driver
                                  'camera_model': 'OAK-D-LITE',
                                  'stereo_fps': '10'
                                  }.items(),
        ),

	Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_imu_tf_publisher',
            arguments=['0', '0', '0.0', '0', '0', '0', '1', 'oak-d-base-frame', 'oak_imu_frame'] # Example: 5cm z offset
        ),

        # Sync right/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/right/image_rect'),
                        ('rgb/camera_info', '/right/camera_info'),
                        ('depth/image', '/stereo/depth')]),

       Node( # Removed imu_filter_madgwick as it's not needed without IMU
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings # No IMU remapping here
            ),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings, # No IMU remapping here
            arguments=['-d']),

        # Visualization
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=parameters,
        #     # Removed: remappings=remappings # No IMU remapping here
        #     )
    ])
