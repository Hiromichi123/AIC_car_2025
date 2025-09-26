from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_path = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(pkg_path, '..', 'worlds', 'empty.world')
    urdf_file = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf')

    return LaunchDescription([
        # 通过 gazebo_ros 启动 world
        ExecuteProcess(
            cmd=['gazebo', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # spawn URDF
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'mycar'],
            output='screen'
        )
    ])
