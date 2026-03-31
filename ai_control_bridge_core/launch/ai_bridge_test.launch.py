"""
Integration Test Launch File for AI Control Bridge + BCR Arm

Tests the complete pipeline:
1. BCR Arm Gazebo simulation
2. Inference processor node
3. Control bridge integration

Run with: ros2 launch ai_control_bridge_core ai_bridge_test.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for integration testing."""
    
    # Get package directories
    bcr_arm_gazebo_dir = get_package_share_directory('bcr_arm_gazebo')
    
    # Launch BCR Arm Gazebo simulation
    bcr_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bcr_arm_gazebo_dir, 'launch', 'bcr_arm.gazebo.launch.py')
        )
    )
    
    # Launch AI Inference Processor Node
    inference_node = Node(
        package='ai_inference_processor',
        executable='ai_inference_processor_node',
        name='ai_inference_processor',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )
    
    # Create launch description
    ld = LaunchDescription([
        bcr_arm_launch,
        inference_node,
    ])
    
    return ld
