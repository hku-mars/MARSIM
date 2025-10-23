#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu_', default_value='false',
        description='Whether to use GPU for rendering'
    )
    
    drone_num_arg = DeclareLaunchArgument(
        'drone_num', default_value='1',
        description='Number of drones'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='world',
        description='Frame ID for the simulation'
    )
    
    global_cloud_topic_arg = DeclareLaunchArgument(
        'global_cloud_topic', default_value='/map_generator/global_cloud',
        description='Topic for global cloud'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/lidar_slam/odom',
        description='Odometry topic'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name', 
        default_value=PathJoinSubstitution([
            FindPackageShare('map_generator'),
            'resource',
            'small_forest01cutoff.pcd'
        ]),
        description='Path to the map file'
    )
    
    downsample_resolution_arg = DeclareLaunchArgument(
        'downsample_resolution', default_value='0.1',
        description='Downsample resolution for the map'
    )
    
    use_uav_extra_model_arg = DeclareLaunchArgument(
        'use_uav_extra_model', default_value='1',
        description='Whether to use UAV extra model'
    )

    # Get launch configurations
    use_gpu = LaunchConfiguration('use_gpu_')
    drone_num = LaunchConfiguration('drone_num')
    frame_id = LaunchConfiguration('frame_id')
    global_cloud_topic = LaunchConfiguration('global_cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    map_name = LaunchConfiguration('map_name')
    downsample_resolution = LaunchConfiguration('downsample_resolution')
    use_uav_extra_model = LaunchConfiguration('use_uav_extra_model')

    # Map generator node - REMOVED (already included in single_drone_simple.launch.py)
    # map_generator_node = Node(
    #     package='map_generator',
    #     executable='map_pub',
    #     name='map_pub',
    #     output='screen',
    #     arguments=[map_name],
    #     parameters=[
    #         {'add_boundary': 0},
    #         {'is_bridge': 0},
    #         {'downsample_res': 0.1},
    #         {'map_offset_x': 0.0},
    #         {'map_offset_y': 0.0},
    #         {'map_offset_z': 0.0},
    #     ]
    # )

    # Single drone launch (using the simple version we created)
    single_drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('test_interface'),
                'launch',
                'single_drone_simple.launch.py'
            ])
        ),
        launch_arguments={
            'drone_id': '0',
            'init_x_': '0.0',
            'init_y_': '0.0',
            'init_z_': '1.0',
            'init_yaw': '0.0',
            'odom_topic': 'odom',
        }.items()
    )

    # RViz node for visualization - REMOVED (already included in single_drone_simple.launch.py)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rvizvisualisation',
    #     output='screen',
    #     arguments=['-d', PathJoinSubstitution([
    #         FindPackageShare('test_interface'),
    #         'config',
    #         'traj.rviz'
    #     ])]
    # )

    return LaunchDescription([
        use_gpu_arg,
        drone_num_arg,
        frame_id_arg,
        global_cloud_topic_arg,
        odom_topic_arg,
        map_name_arg,
        downsample_resolution_arg,
        use_uav_extra_model_arg,
        # map_generator_node,  # REMOVED - already in single_drone_simple.launch.py
        single_drone_launch,
        # rviz_node,  # REMOVED - already in single_drone_simple.launch.py
    ])