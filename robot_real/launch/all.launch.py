from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 建图
    slam_launch = os.path.join(
        get_package_share_directory('robot_real'),
        'slam.launch.py'
    )

    # 工具
    ros2_tools_launch = os.path.join(
        get_package_share_directory('ros2_tools'),
        'launch',
        'tools_real.launch.py'
    )

    # 视觉
    vision_node = Node(
        package='vision_node',
        executable='vision_real',
        name='vision_real',
        output='screen'
    )

    # 导航
    navi_node = Node(
        package='navi_rs',
        executable='navi_rs',
        name='navi_rs',
        output='screen'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch)
    )

    ros2_tools = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_tools_launch)
    )
    
    return LaunchDescription([
        slam,
        ros2_tools,
        TimerAction(period=10.0, actions=[vision_node]),
        TimerAction(period=10.0, actions=[navi_node]),
    ])
