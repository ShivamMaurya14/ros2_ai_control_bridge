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
    Launch AI Control Bridge for REAL BCR ARM robot hardware.
    
    This launch file orchestrates the complete system:
    1. Hardware interface (communicates with real robot)
    2. Control loop (ros2_control plugin @ 1000 Hz)
    3. Inference processor (neural network policy @ 50 Hz)
    4. Monitoring/diagnostics
    5. RViz for visualization (optional)
    """
    
    # Declare launch arguments
    launch_args = []
    
    launch_args.append(DeclareLaunchArgument(
        'robot_config',
        default_value='bcr_arm_real.yaml',
        description='Robot configuration file (in config/ directory)'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'controller_config',
        default_value='bcr_arm_real.yaml',
        description='Controller parameter file'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'hardware_interface',
        default_value='real',
        description='Hardware interface type: real or sim'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging'
    ))
    
    # Get package share directories
    ai_bridge_share = FindPackageShare('ai_control_bridge_controller')
    bcr_arm_share = FindPackageShare('bcr_arm_description')
    
    # Build config file paths
    config_file = PathJoinSubstitution([
        ai_bridge_share,
        'config',
        LaunchConfiguration('controller_config')
    ])
    
    robot_description_file = PathJoinSubstitution([
        bcr_arm_share,
        'urdf',
        'bcr_arm.urdf.xacro'
    ])
    
    # Nodes to launch
    nodes = []
    
    # 1. Robot State Publisher (publishes TF tree)
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_file,
            'use_sim_time': False,  # Real hardware uses wall clock
        }],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    ))
    
    # 2. Joint State Aggregator
    # Reads from hardware and publishes combined /joint_states
    nodes.append(Node(
        package='joint_state_broadcaster',
        executable='joint_state_broadcaster',
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    ))
    
    # 3. Hardware Interface Node
    # Connects to actual robot hardware (CAN/EtherCAT/USB)
    nodes.append(Node(
        package='ai_control_bridge_hardware',
        executable='hardware_interface_node',
        parameters=[config_file, {
            'hardware_type': LaunchConfiguration('hardware_interface'),
        }],
        output='screen',
        prefix='taskset -c 0',  # Pin to CPU 0 for consistency
        respawn=True,
        respawn_delay=5.0
    ))
    
    # 4. Controller Manager (ros2_control plugin loader)
    nodes.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    ))
    
    # 5. AI Control Bridge Controller (roscontrol plugin)
    # This runs in the control loop at 1000 Hz
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ai_control_bridge_controller', '-c', '/controller_manager'],
        output='screen'
    ))
    
    # 6. Inference Processor Node (50 Hz async)
    nodes.append(Node(
        package='ai_inference_processor',
        executable='ai_inference_processor_node',
        parameters=[config_file, {
            'log_level': 'INFO' if not IfCondition(LaunchConfiguration('verbose')).evaluate(None) else 'DEBUG',
        }],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    ))
    
    # 7. Diagnostics/Monitoring Node
    nodes.append(Node(
        package='ai_control_bridge_core',
        executable='diagnostics_node',
        parameters=[config_file],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    ))
    
    # 8. RViz Visualization (optional)
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            ai_bridge_share,
            'config',
            'navigation.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    ))
    
    return LaunchDescription(launch_args + nodes)
