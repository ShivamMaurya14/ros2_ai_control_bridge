# Copyright 2026 GSoC Contributors
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Launch AI Control Bridge in Gazebo SIMULATION.
    
    Complete testing environment with:
    1. Gazebo physics simulator
    2. Control loop (ros2_control plugin @ 1000 Hz)
    3. Inference processor (neural network policy @ 50 Hz)
    4. RViz visualization
    5. Diagnostics/monitoring
    """
    
    # Declare launch arguments
    launch_args = []
    
    launch_args.append(DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo in paused mode'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo clock'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging'
    ))
    
    # Get package share directories
    ai_bridge_share = FindPackageShare('ai_control_bridge_controller')
    bcr_arm_share = FindPackageShare('bcr_arm_gazebo')
    
    # Build config file paths
    controller_config = PathJoinSubstitution([
        ai_bridge_share,
        'config',
        'bcr_arm_gazebo.yaml'
    ])
    
    gazebo_world = PathJoinSubstitution([
        bcr_arm_share,
        'worlds',
        'default.world'
    ])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ]),
        launch_arguments={
            'world': gazebo_world,
            'paused': LaunchConfiguration('paused'),
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # BCR ARM Gazebo spawn
    bcr_arm_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            bcr_arm_share,
            'launch',
            'bcr_arm.gazebo.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # Nodes to launch
    nodes = []
    
    # 1. Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    ))
    
    # 2. Joint State Aggregator
    nodes.append(Node(
        package='joint_state_broadcaster',
        executable='joint_state_broadcaster',
        parameters=[controller_config],
        output='screen'
    ))
    
    # 3. AI Control Bridge Controller (main control loop)
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ai_control_bridge_controller', '-c', '/controller_manager'],
        output='screen'
    ))
    
    # 4. Inference Processor Node (50 Hz neural network)
    nodes.append(Node(
        package='ai_inference_processor',
        executable='ai_inference_processor_node',
        parameters=[controller_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    ))
    
    # 5. Diagnostics Node (real-time monitoring)
    nodes.append(Node(
        package='ai_control_bridge_core',
        executable='diagnostics_node',
        parameters=[controller_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    ))
    
    # 6. RViz Visualization
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            ai_bridge_share,
            'config',
            'simulation.rviz'
        ]), '--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    ))
    
    return LaunchDescription(launch_args + [gazebo_launch, bcr_arm_gazebo] + nodes)
