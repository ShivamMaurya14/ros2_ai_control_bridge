#!/usr/bin/env python3
# Copyright 2026 GSoC Contributors
# Licensed under the Apache License, Version 2.0

"""
AI Control Bridge + BCR ARM - Gazebo Simulation Integration

This launch file brings together:
1. BCR ARM in Gazebo physics simulator
2. AI Control Bridge controller @ 1000 Hz
3. Inference processor (policy @ 50 Hz)
4. RViz visualization
5. Diagnostics monitoring

Usage:
  ros2 launch ai_control_bridge_controller bcr_arm_gazebo_integrated.launch.py
  
Optional arguments:
  paused:=true          - Start simulation paused
  gui:=false            - Launch Gazebo without GUI
  launch_rviz:=false    - Don't show RViz
  use_sim_time:=true    - Use Gazebo clock (recommended)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Generate launch description for AI Control Bridge + BCR ARM in Gazebo.
    """
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    
    launch_args = []
    
    # Gazebo simulation control
    launch_args.append(DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo in paused mode (useful for debugging)'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo with GUI (set to false for headless operation)'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock (critical for timing)'
    ))
    
    # RViz visualization
    launch_args.append(DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    ))
    
    # BCR ARM configuration
    launch_args.append(DeclareLaunchArgument(
        'robot_name',
        default_value='bcr_arm',
        description='Name of the robot'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Robot joint/link prefix (usually empty for single robot)'
    ))
    
    # =========================================================================
    # PACKAGE PATHS
    # =========================================================================
    
    ai_bridge_share = FindPackageShare('ai_control_bridge_controller')
    bcr_arm_desc_share = FindPackageShare('bcr_arm_description')
    bcr_arm_gazebo_share = FindPackageShare('bcr_arm_gazebo')
    bcr_arm_moveit_share = FindPackageShare('bcr_arm_moveit_config')
    
    # =========================================================================
    # GENERATE ROBOT URDF
    # =========================================================================
    
    # Build the complete robot description with Gazebo plugins
    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            bcr_arm_desc_share,
            'urdf',
            'robots',
            'bcr_arm.urdf.xacro'
        ]),
        ' ',
        'robot_name:=',
        LaunchConfiguration('robot_name'),
        ' ',
        'prefix:=',
        LaunchConfiguration('prefix'),
        ' ',
        'use_gazebo:=true',  # Enable Gazebo plugins
        ' ',
        'add_world:=true',
        ' ',
        'base_link:=base_link',
        ' ',
        'base_type:=g_shape',
        ' ',
        'flange_link:=link6_flange',
        ' ',
        'gripper_type:=adaptive_gripper',
        ' ',
        'use_camera:=false',
        ' ',
        'ros2_controllers_path:=',
        PathJoinSubstitution([
            bcr_arm_moveit_share,
            'config',
            'ai_control_bridge_controllers.yaml'  # Use AI-integrated config
        ])
    ])
    
    robot_description = ParameterValue(
        robot_description_command,
        value_type=str
    )
    
    # =========================================================================
    # ROBOT STATE PUBLISHER
    # =========================================================================
    # Publishes tf tree (robot kinematics) for RViz and other tools
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # =========================================================================
    # GAZEBO SIMULATION
    # =========================================================================
    
    gazebo_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [
                '-r ',  # Run immediately (don't pause)
                '-v 4 ',  # Verbosity level
                PathJoinSubstitution([
                    bcr_arm_gazebo_share,
                    'worlds',
                    'empty.world'
                ])
            ]
        }.items()
    )
    
    # =========================================================================
    # SPAWN ROBOT IN GAZEBO
    # =========================================================================
    
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_command,
            '-name', LaunchConfiguration('robot_name'),
            '-allow_renaming', 'true',
        ]
    )
    
    # =========================================================================
    # CONTROLLER SPAWNING
    # =========================================================================
    
    # Spawn joint state broadcaster (publishes /joint_states)
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )
    
    # Spawn AI Control Bridge controller (1000 Hz control loop)
    spawn_ai_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ai_control_bridge_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )
    
    # Event handler: spawn joint state broadcaster after robot is spawned
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[spawn_joint_state_broadcaster]
        )
    )
    
    # Event handler: spawn AI controller after joint state broadcaster is ready
    delay_ai = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_ai_controller]
        )
    )
    
    # =========================================================================
    # INFERENCE PROCESSOR NODE
    # =========================================================================
    # Runs neural network policy @ 50 Hz and publishes waypoints
    
    inference_node = Node(
        package='ai_inference_processor',
        executable='ai_inference_processor_node',
        name='ai_inference_processor',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                ai_bridge_share,
                'config',
                'bcr_arm_gazebo.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # =========================================================================
    # DIAGNOSTICS NODE
    # =========================================================================
    # Monitors real-time performance, constraint violations, etc.
    
    diagnostics_node = Node(
        package='ai_control_bridge_core',
        executable='diagnostics_node',
        name='diagnostics_monitor',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                ai_bridge_share,
                'config',
                'bcr_arm_gazebo.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # =========================================================================
    # RVIZ VISUALIZATION
    # =========================================================================
    
    # Create RViz config path
    rviz_config_path = PathJoinSubstitution([
        ai_bridge_share,
        'config',
        'simulation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    
    return LaunchDescription([
        # Arguments must come first
        *launch_args,
        
        # Core simulation components
        robot_state_publisher_node,
        gazebo_sim_node,
        spawn_entity_node,
        
        # Controller spawning with sequencing
        delay_jsb,
        delay_ai,
        
        # AI integration nodes
        inference_node,
        diagnostics_node,
        
        # Visualization
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
