from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 lidar_data_node
        Node(
            package='ros2_tools',
            executable='lidar_data_node',
            name='lidar_data_node',
            output='screen'
        ),
        # 启动 bsp_node
        Node(
            package='ros2_tools',
            executable='bsp_node',
            name='bsp_node',
            output='screen'
        ),
    ])