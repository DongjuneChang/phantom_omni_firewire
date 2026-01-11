"""
Phantom Omni ROS2 Launch File

FireWire version - for Sensable Phantom Omni haptic device

Usage:
  ros2 launch omni_common omni_state.launch.py              # Driver only
  ros2 launch omni_common omni_state.launch.py rviz:=true   # With RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    omni_common_dir = get_package_share_directory('omni_common')
    omni_description_dir = get_package_share_directory('omni_description')

    # Get URDF file path
    urdf_file = os.path.join(omni_description_dir, 'urdf', 'omni.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Get config/rviz file paths
    config_file = os.path.join(omni_common_dir, 'config', 'phantom_omni.yaml')
    rviz_file = os.path.join(omni_common_dir, 'rviz', 'phantom_default.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config YAML file'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz'
        ),

        # Robot State Publisher - publishes TF from URDF + joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
            }],
            remappings=[
                ('joint_states', '/phantom/joint_states'),
            ]
        ),

        # Phantom Omni driver - uses YAML config for joint_scales/offsets
        Node(
            package='omni_common',
            executable='omni_state',
            name='omni_haptic_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),

        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

