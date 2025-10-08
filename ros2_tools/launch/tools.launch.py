from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
import os

def generate_launch_description():

    # 5. 启动 lidar_data_node (转换 Gazebo odom 到 LidarPose)
    lidar_data_node = Node(
        package='ros2_tools',
        executable='lidar_data_node',
        name='lidar_data_node',
        output='screen'
    )

    # 6. 启动 bsp_node (位置控制器)
    bsp_node = Node(
        package='ros2_tools',
        executable='bsp_node',
        name='bsp_node',
        output='screen'
    )

    # # 7. 启动 navi_rs (导航节点)
    # navi_node = ExecuteProcess(
    #     cmd=['ros2', 'run', 'navi_rs', 'navi_rs'],
    #     output='screen'
    # )

    return LaunchDescription([
        lidar_data_node,
        bsp_node,
        # navi_node
    ])