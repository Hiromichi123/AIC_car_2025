"""
仿真环境下启动LidarDataNode和BSP节点的launch文件
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