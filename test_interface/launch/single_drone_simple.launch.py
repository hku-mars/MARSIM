#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='0', description='Drone ID')
    init_x_arg = DeclareLaunchArgument('init_x_', default_value='0.0', description='Initial X position')
    init_y_arg = DeclareLaunchArgument('init_y_', default_value='0.0', description='Initial Y position')
    init_z_arg = DeclareLaunchArgument('init_z_', default_value='1.0', description='Initial Z position')
    init_yaw_arg = DeclareLaunchArgument('init_yaw', default_value='0.0', description='Initial yaw angle')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='odom', description='Odometry topic')

    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    init_x = LaunchConfiguration('init_x_')
    init_y = LaunchConfiguration('init_y_')
    init_z = LaunchConfiguration('init_z_')
    init_yaw = LaunchConfiguration('init_yaw')
    odom_topic = LaunchConfiguration('odom_topic')

    # Quadrotor dynamics node
    quadrotor_dynamics_node = Node(
        package='mars_drone_sim',
        executable='quadrotor_dynamics_node',
        name='quadrotor_dynamics_node',
        output='screen',
        parameters=[{
            'mass': 1.9,
            'simulation_rate': 200.0,
            'quadrotor_name': 'quadrotor',
            'init_state_x': init_x,
            'init_state_y': init_y,
            'init_state_z': init_z,
        }]
    )

    # Cascade PID controller node
    cascade_pid_node = Node(
        package='cascadePID',
        executable='cascadePID_node',
        name='cascade_pid_node',
        output='screen',
        parameters=[{
            'drone_id': drone_id,
            'controller_rate': 200.0,
            'quadrotor_name': 'quadrotor',
            'init_state_x': init_x,
            'init_state_y': init_y,
            'init_state_z': init_z,
            'init_state_yaw': init_yaw,
            'angle_stable_time': 0.5,
            'damping_ratio': 1.0,
        }]
    )

    # Map generator node
    map_generator_node = Node(
        package='map_generator',
        executable='map_pub',
        name='map_pub',
        output='screen',
        arguments=['/home/jaeyoung/ros2_ws/src/MARSIM/map_generator/resource/small_forest01cutoff.pcd'],
        parameters=[{
            'add_boundary': 0,
            'is_bridge': 0,
            'downsample_res': 0.1,
            'map_offset_x': 0.0,
            'map_offset_y': 0.0,
            'map_offset_z': 0.0,
        }]
    )

    # Test interface node
    test_interface_node = Node(
        package='test_interface',
        executable='test_interface_node',
        name='test_interface_node',
        output='screen'
    )

    # LiDAR simulation node
    lidar_node = Node(
        package='local_sensing_node',
        executable='opengl_render_node',
        name='quad0_pcl_render_node',
        output='screen',
        arguments=['/home/jaeyoung/ros2_ws/src/MARSIM/map_generator/resource/small_forest01cutoff.pcd'],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', '/odom'),
        ],
        parameters=[{
            'drone_id': drone_id,
            'quadrotor_name': 'quad_0',
            'uav_num': 1,
            'is_360lidar': 0,
            'sensing_horizon': 30.0,
            'sensing_rate': 10.0,
            'estimation_rate': 10.0,
            'polar_resolution': 0.2,
            'yaw_fov': 70.4,
            'vertical_fov': 77.2,
            'min_raylength': 1.0,
            'livox_linestep': 1.4,
            'curvature_limit': 100.0,
            'hash_cubesize': 5.0,
            'use_avia_pattern': 1,
            'downsample_res': 0.1,
            'dynobj_enable': 0,
            'dynobject_size': 0.5,
            'dynobject_num': 5,
            'dyn_mode': 1,
            'dyn_velocity': 1.0,
            'use_uav_extra_model': 1,
            'collisioncheck_enable': 0,
            'collision_range': 0.5,
            'output_pcd': 0,
            'uav_num': 1,
        }],
        env={'DISPLAY': ':0'}
    )

    # Odom visualization node for 3D drone model and trajectory
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        parameters=[{
            'mesh_resource': 'package://odom_visualization/meshes/yunque.dae',
            'color_r': 1.0,
            'color_g': 0.0,
            'color_b': 0.0,
            'color_a': 1.0,
            'robot_scale': 2.0,
            'frame_id': 'world',
            'drone_id': drone_id,
            'quadrotor_name': 'quadrotor',
        }],
        remappings=[
            ('odom', 'odom'),
        ]
    )

    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('test_interface'),
            'config',
            'traj_simple.rviz'
        ])]
    )

    return LaunchDescription([
        drone_id_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        init_yaw_arg,
        odom_topic_arg,
        quadrotor_dynamics_node,
        cascade_pid_node,
        map_generator_node,
        test_interface_node,
        lidar_node,  # LiDAR simulation - re-enabled
        odom_visualization_node,  # 3D drone model and trajectory
        rviz_node,  # RViz visualization
    ])
