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

import copy

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
         'frame_id':'base_footprint',
         'subscribe_rgbd':True,
         'subscribe_odom_info':True,
         'approx_sync':True,
         'approx_sync_max_interval': 0.1,
         'wait_imu_to_init':True,
         'Reg/ForceOdometryUpdate': 'true',
         'Odom/ResetCooldown': '1',
         'Rtabmap/StartNewMapOnLoopClosure': 'true',

         'Optimizer/GravitySigma': '0.3',
         'Vis/ForceIMUInitialization': 'true',

         'Odom/Noise/Angular': "0.1",
         'Odom/Noise/Linear': "0.05",
         'Optimizer/Strategy': "1",

         'Vis/CorType': '1',
         'Vis/FeatureType': '6',
         'Vis/MaxFeatures': '1000',
	     'Vis/MinInliers': '12',
	     'Vis/Pipeline': '6',
         'Mem/ReduceGraph': 'true',
         'OdomF2M/MaxSize': '1000',
         'cloud_noise_filtering_radius': '0.05',
         'cloud_noise_filtering_min_neighbors': '10',
         'proj_max_ground_angle': '45',
         'proj_max_ground_height': '0.1',
         'Reg/Force3DoF': 'true',
         'Optimizer/Slam2D': 'true',

         'RGBD/NeighborLinkRefining': 'true',
         'RGBD/ProximityBySpace': 'false',
         'RGBD/AngularUpdate': '0.01',
         'RGBD/LinearUpdate': '0.1',
         'RGBD/OptimizeFromGraphEnd': 'false',
         'RGBD/OptimizeStrategy': '2',
         'RGBD/OptimizeRobust': 'true',
         'RGBD/OptimizeMaxError': '0',

         
         'publish_tf': True,

         'database_path': '~/.ros/rtabmap.db',
         'rtabmap_args': '--delete_db_on_start',
         'Rtabmap/StartNewMap': 'true',
         'Rtabmap/Mem/Incremental': 'true',

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
                launch_arguments={
                                  'parent_frame': 'camera_link',
                                  'depth_aligned': 'true', # Für die Befähigung des Modells
                                  'enableRviz': 'false', # Keine visualisation
                                  'monoResolution': '400p', # Auflösung der Stereo Kameras
				                  'rgbResolution': '1080p', # Auflösung der Farbkamera
                                  'rgbScaleNumerator': '1', # Zähler des Skalars der Auflösung der Farbkamera
                                  'rgbScaleDinominator': '2', # Nenner des Skalars der Auflösung der Farbkamera
                                  'camera_model': 'OAK-D-LITE', # Definition des Kameramodells
                                  'stereo_fps': '5', # Aktualisierungsrate der Tiefenbilder
                                  'nnName': 'butter_person_v1_openvino_2022.1_3shave.blob', # Name des Objekterkennungs Modell
                                  'resourceBaseFolder': '/home/leosc/MaturaProject/PassTheButterRobot/result', # Lokaler Pfad zum Modell
                                  'detectionClassesCount': '2', # Anzahl Klassen im Modell
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
