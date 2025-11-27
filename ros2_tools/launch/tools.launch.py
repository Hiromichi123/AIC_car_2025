"""
Launch file for simulation mode.

This launch file starts the necessary nodes for running in Gazebo simulation:
- lidar_data_node: Converts Gazebo odometry to LidarPose format
- bsp_node: Position controller for goal tracking

Usage:
  ros2 launch ros2_tools tools.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # LidarDataNode - processes Gazebo odometry for simulation
    lidar_data_node = Node(
        package='ros2_tools',
        executable='lidar_data_node',
        name='lidar_data_node',
        output='screen',
        parameters=[{
            'use_simulation': True,
            'simulation_odom_topic': '/absolute_pose',
            'real_robot_odom_topic': '/aft_mapped_to_init',
        }]
    )

    # BSP Node - position controller
    bsp_node = Node(
        package='ros2_tools',
        executable='bsp_node',
        name='bsp_node',
        output='screen'
    )

    return LaunchDescription([
        lidar_data_node,
        bsp_node,
    ])