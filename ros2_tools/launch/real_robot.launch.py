"""
Launch file for real robot mode.

This launch file starts the necessary nodes for running on the real robot hardware:
- lidar_data_node: Converts PointLIO odometry to LidarPose format
- bsp_node: Position controller for goal tracking
- hardware_bridge_node: Serial communication bridge to motor controller

Usage:
  ros2 launch ros2_tools real_robot.launch.py
  
With custom serial port:
  ros2 launch ros2_tools real_robot.launch.py serial_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor controller communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    enable_serial_arg = DeclareLaunchArgument(
        'enable_serial',
        default_value='true',
        description='Enable serial communication (set to false for testing)'
    )

    # LidarDataNode - processes PointLIO odometry for real robot
    lidar_data_node = Node(
        package='ros2_tools',
        executable='lidar_data_node',
        name='lidar_data_node',
        output='screen',
        parameters=[{
            'use_simulation': False,
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

    # Hardware Bridge Node - serial communication to motor controller
    hardware_bridge_node = Node(
        package='ros2_tools',
        executable='hardware_bridge_node',
        name='hardware_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'enable_serial': LaunchConfiguration('enable_serial'),
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
            'wheel_radius': 0.05,
            'robot_length': 0.3,
            'robot_width': 0.25,
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        enable_serial_arg,
        lidar_data_node,
        bsp_node,
        hardware_bridge_node,
    ])
