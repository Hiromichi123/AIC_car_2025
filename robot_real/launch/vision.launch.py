from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 视觉
    vision_node = Node(
        package='vision_node',
        executable='vision_real',
        name='vision_real',
        output='screen'
    )

    # yolip
    yolip_node = Node(
        package='yolip',
        executable='yolip',
        name='yolip',
        output='screen'
    )

    return LaunchDescription([
        vision_node,
        yolip_node,
    ])
