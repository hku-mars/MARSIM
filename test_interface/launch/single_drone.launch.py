#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument('drone_id', description='Drone ID')
    uav_num_arg = DeclareLaunchArgument('uav_num_', description='Number of UAVs')
    init_x_arg = DeclareLaunchArgument('init_x_', description='Initial X position')
    init_y_arg = DeclareLaunchArgument('init_y_', description='Initial Y position')
    init_z_arg = DeclareLaunchArgument('init_z_', description='Initial Z position')
    init_yaw_arg = DeclareLaunchArgument('init_yaw', description='Initial yaw angle')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', description='Odometry topic')
    lidar_type_arg = DeclareLaunchArgument('lidar_type', description='LiDAR type')
    map_name_arg = DeclareLaunchArgument('map_name_', description='Map file name')
    downsample_resolution_arg = DeclareLaunchArgument('downsample_resolution_', description='Downsample resolution')
    use_gpu_arg = DeclareLaunchArgument('use_gpu', description='Use GPU for rendering')
    use_uav_extra_model_arg = DeclareLaunchArgument('use_uav_extra_model_', description='Use extra UAV model')

    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    uav_num = LaunchConfiguration('uav_num_')
    init_x = LaunchConfiguration('init_x_')
    init_y = LaunchConfiguration('init_y_')
    init_z = LaunchConfiguration('init_z_')
    init_yaw = LaunchConfiguration('init_yaw')
    odom_topic = LaunchConfiguration('odom_topic')
    lidar_type = LaunchConfiguration('lidar_type')
    map_name = LaunchConfiguration('map_name_')
    downsample_resolution = LaunchConfiguration('downsample_resolution_')
    use_gpu = LaunchConfiguration('use_gpu')
    use_uav_extra_model = LaunchConfiguration('use_uav_extra_model_')

    # Quadrotor dynamics node
    quadrotor_dynamics_node = Node(
        package='mars_drone_sim',
        executable='quadrotor_dynamics_node',
        name=PythonExpression(["'quad' + str(", drone_id, ") + '_quadrotor_dynamics'"]),
        output='log',
        remappings=[
            ('~/odom', PythonExpression(["'/quad_' + str(", drone_id, ") + '/' + str(", odom_topic, ")"])),
            ('~/cmd_RPM', PythonExpression(["'/quad_' + str(", drone_id, ") + '/cmdRPM'"])),
            ('~/imu', PythonExpression(["'/quad_' + str(", drone_id, ") + '/imu'"])),
        ],
        parameters=[{
            'mass': 1.9,
            'simulation_rate': 200,
            'quadrotor_name': ['quad_', drone_id],
            'init_state_x': init_x,
            'init_state_y': init_y,
            'init_state_z': init_z,
        }]
    )

    # Cascade PID node
    cascade_pid_node = Node(
        package='cascadePID',
        executable='cascadePID_node',
        name=['quad', drone_id, '_cascadePID_node'],
        remappings=[
            ('~/odom', ['/quad_', drone_id, '/', odom_topic]),
            ('~/cmd_RPM', ['/quad_', drone_id, '/cmdRPM']),
            ('~/cmd_pose', '/goal'),
            ('~/position_cmd', ['/quad_', drone_id, '/planning/pos_cmd']),
        ],
        parameters=[{
            'drone_id': drone_id,
            'controller_rate': 200,
            'quadrotor_name': ['quad_', drone_id],
            'init_state_x': init_x,
            'init_state_y': init_y,
            'init_state_z': init_z,
            'init_state_yaw': init_yaw,
            'angle_stable_time': 0.5,
            'damping_ratio': 1.0,
        }]
    )

    # Odometry visualization node
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['quad', drone_id, '_odom_visualization'],
        output='screen',
        remappings=[
            ('~/odom', ['/quad_', drone_id, '/', odom_topic]),
            ('~/cmd', ['/quad_', drone_id, '/pos_cmd']),
        ],
        parameters=[{
            'init_x': init_x,
            'init_y': init_y,
            'init_z': init_z,
            'color/a': 1.0,
            'color/r': 0.0,
            'color/g': 1.0,
            'color/b': 1.0,
            'covariance_scale': 100.0,
            'robot_scale': 1.0,
            'quadrotor_name': ['quad', drone_id],
            'drone_id': drone_id,
            'mesh_resource': 'package://odom_visualization/meshes/yunque.dae',
        }]
    )

    # GPU-based sensing nodes
    gpu_avia_sensing = GroupAction(
        condition=IfCondition(use_gpu),
        actions=[
            GroupAction(
                condition=IfCondition(PythonExpression(['"', lidar_type, '" == "avia"'])),
                actions=[
                    Node(
                        package='local_sensing_node',
                        executable='opengl_render_node',
                        name=['quad', drone_id, '_pcl_render_node'],
                        output='screen',
                        arguments=[map_name],
                        remappings=[
                            ('~/global_map', '/map_generator/global_cloud'),
                            ('~/odometry', ['/quad_', drone_id, '/', odom_topic]),
                        ],
                        parameters=[{
                            'drone_id': drone_id,
                            'quadrotor_name': ['quad_', drone_id],
                            'uav_num': uav_num,
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
                            'downsample_res': downsample_resolution,
                            'dynobj_enable': 0,
                            'dynobject_size': 0.8,
                            'dynobject_num': 10,
                            'dyn_mode': 0,
                            'dyn_velocity': 1.0,
                            'use_uav_extra_model': use_uav_extra_model,
                            'collisioncheck_enable': 0,
                            'collision_range': 0.3,
                            'output_pcd': 0,
                        }]
                    )
                ]
            ),
            GroupAction(
                condition=IfCondition(PythonExpression(['"', lidar_type, '" == "mid360"'])),
                actions=[
                    Node(
                        package='local_sensing_node',
                        executable='opengl_render_node',
                        name=['quad', drone_id, '_pcl_render_node'],
                        output='screen',
                        arguments=[map_name],
                        remappings=[
                            ('~/global_map', '/map_generator/global_cloud'),
                            ('~/odometry', ['/quad_', drone_id, '/', odom_topic]),
                        ],
                        parameters=[{
                            'drone_id': drone_id,
                            'quadrotor_name': ['quad_', drone_id],
                            'uav_num': uav_num,
                            'is_360lidar': 1,
                            'sensing_horizon': 15.0,
                            'sensing_rate': 10.0,
                            'estimation_rate': 10.0,
                            'polar_resolution': 0.2,
                            'yaw_fov': 360.0,
                            'vertical_fov': 90.0,
                            'min_raylength': 1.0,
                            'livox_linestep': 1.4,
                            'curvature_limit': 100.0,
                            'hash_cubesize': 5.0,
                            'use_avia_pattern': 0,
                            'use_vlp32_pattern': 0,
                            'use_minicf_pattern': 1,
                            'downsample_res': downsample_resolution,
                            'dynobj_enable': 0,
                            'dynobject_size': 0.8,
                            'dynobject_num': 10,
                            'dyn_mode': 0,
                            'dyn_velocity': 1.0,
                            'use_uav_extra_model': use_uav_extra_model,
                            'collisioncheck_enable': 0,
                            'collision_range': 0.3,
                            'output_pcd': 0,
                        }]
                    )
                ]
            )
        ]
    )

    # Non-GPU sensing nodes
    no_gpu_sensing = GroupAction(
        condition=UnlessCondition(use_gpu),
        actions=[
            GroupAction(
                condition=IfCondition(PythonExpression(['"', lidar_type, '" == "mid360"'])),
                actions=[
                    Node(
                        package='local_sensing_node',
                        executable='pcl_render_node',
                        name=['quad', drone_id, '_pcl_render_node'],
                        output='screen',
                        remappings=[
                            ('~/global_map', '/map_generator/global_cloud'),
                            ('~/odometry', ['/quad_', drone_id, '/', odom_topic]),
                        ],
                        parameters=[{
                            'drone_id': drone_id,
                            'quadrotor_name': ['quad_', drone_id],
                            'uav_num': uav_num,
                            'is_360lidar': 1,
                            'sensing_horizon': 15.0,
                            'sensing_rate': 10.0,
                            'estimation_rate': 10.0,
                            'polar_resolution': 0.2,
                            'yaw_fov': 360.0,
                            'vertical_fov': 90.0,
                            'min_raylength': 1.0,
                            'livox_linestep': 1.4,
                            'curvature_limit': 100.0,
                            'hash_cubesize': 5.0,
                            'use_avia_pattern': 0,
                            'use_vlp32_pattern': 0,
                            'use_minicf_pattern': 1,
                            'downsample_res': downsample_resolution,
                            'dynobj_enable': 0,
                            'dynobject_size': 0.6,
                            'dynobject_num': 30,
                            'dyn_mode': 2,
                            'dyn_velocity': 3.0,
                            'use_uav_extra_model': use_uav_extra_model,
                            'collisioncheck_enable': 0,
                            'collision_range': 0.3,
                            'output_pcd': 0,
                        }]
                    )
                ]
            )
        ]
    )

    return LaunchDescription([
        drone_id_arg,
        uav_num_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        init_yaw_arg,
        odom_topic_arg,
        lidar_type_arg,
        map_name_arg,
        downsample_resolution_arg,
        use_gpu_arg,
        use_uav_extra_model_arg,
        quadrotor_dynamics_node,
        cascade_pid_node,
        odom_visualization_node,
        gpu_avia_sensing,
        no_gpu_sensing,
    ])
