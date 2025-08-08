# Requirements:
#   A OAK-D camera
#   Install depthai-ros package (https://github.com/luxonis/depthai-ros)
# Example:
#   $ ros2 launch rtabmap_examples depthai.launch.py camera_model:=OAK-D

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    mode = LaunchConfiguration('mapping', default=True)
    declare_mode = DeclareLaunchArgument('mapping', default_value=mode)

    startNewMap = None
    rtabmap_arg = None
    if mode == 'localisation':
        startNewMap = False
        rtabmap_arg = ''
    else:
        startNewMap = True
        rtabmap_arg = '--delete_db_on_start'

    parameters=[{
         'frame_id':'base_link',
         'subscribe_rgbd':True,
        #  'subscribe_rgb': True,
        #  'subscribe_depth': True,
         'subscribe_odom_info':True,
         'approx_sync':True,
         'approx_sync_max_interval': 0.1,
         'wait_imu_to_init':True,
         'Reg/ForceOdometryUpdate': 'true',
         'Odom/ResetCountdown': '0',
         'Odom/Strategy': '0',
	     'Odom/ImageDecimation': '2',
         'Optimizer/GravitySigma': '0.3',
         'Vis/ForceIMUInitialization': 'true',


         'Vis/FeatureType': '6',
         'Vis/MaxFeatures': '1500',
	     'Vis/MinInliers': '12',
	     'Vis/Pipeline': '6',
         'Mem/ReduceGraph': 'true',
         
         'Rtabmap/KeyFrameThr': '0.4',
         
         'QueueSize': 10,
         'Rtabmap/PublishTF': 'true',

         'database_path': '~/.ros/rtabmap.db',
         'rtabmap_args': rtabmap_arg,
         'Rtabmap/StartNewMap': startNewMap,
         'Rtabmap/Mem/Incremental': startNewMap,

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
         'Rtabmap/SavePointCloud': 'false',
	     'Rtabmap/SaveImgRaw': 'false',
	     'Rtabmap/SaveDepthRaw': 'false'
	    }]

    remappings=[('imu', '/imu/data')]

    return LaunchDescription([
        declare_mode,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ptbr'), 'launch', 'rsp.launch.py'))
        ),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py']),
                launch_arguments={'usb2Mode': 'true',
                                  'depth_aligned': 'true',
                                  'enableRviz': 'false',
                                  'monoResolution': '480p',
				                  'rgbResolution': '4K',
                                  'rgbScaleNumerator': '1',
                                  'rgbScaleDinominator': '5',
                                  'enable_imu': 'true', # Added: explicitly disable IMU in the camera driver
                                  'camera_model': 'OAK-D-LITE',
                                  'stereo_fps': '10',
                                  'nnName': 'butter_person_v1_openvino_2022.1_3shave.blob',
                                  'resourceBaseFolder': '/home/leosc/MaturaProject/PassTheButterRobot/result',
                                  'detectionClassesCount': '2',
                                  'parent_frame': 'camera_link',
                                #   'previewHeight': '640',
                                #   'previewWidth': '640'
                                  }.items(),
        ),

	    Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_imu_tf_publisher',
            arguments=['0', '0', '0.0', '0.7071', '0', '0', '0.7071', 'camera_link', 'oak_imu_frame'] # Example: 5cm z offset
        ),
        # Sync right/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/color/image'),
                        ('rgb/camera_info', '/color/camera_info'),
                        ('depth/image', 'stereo/converted_depth')]),

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
