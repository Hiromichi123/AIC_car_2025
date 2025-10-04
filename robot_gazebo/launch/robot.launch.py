from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(pkg_path, '..', 'worlds', 'empty.world')
    urdf_xacro = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf.xacro')
    urdf_file = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf')
    yaml_file = os.path.join(pkg_path, '..', 'urdf', 'mecanum_controllers.yaml')

    # xacro 生成 urdf
    xacro_urdf_process = ExecuteProcess(
        cmd=['ros2', 'run', 'xacro', 'xacro', urdf_xacro, '-o', urdf_file],
        output='screen'
    )

    # 发布 robot_description
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_xacro])}]
    )

    # 启动 Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', 
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            world_file,
        ],
        output='screen'
    )

    # 使用参数服务器加载URDF
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mycar',
            '-x', '0.00',
            '-y', '0.00',
            '-z', '0.02'
        ],
        output='screen'
    )
    spawn_entity_node = TimerAction(period=5.0, actions=[spawn_entity_node]) # 延迟5秒启动

    mecanum_controller_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'mecanum_controller',
            '--param', '/home/hiro/ros2/AIC_car_2025/robot_gazebo/urdf/mecanum_controllers.yaml'
        ],
        output='screen'
    )
    mecanum_controller_spawner = TimerAction(period=10.0, actions=[mecanum_controller_spawner]) # 延迟10秒启动

    return LaunchDescription([
        xacro_urdf_process,
        robot_description,
        gazebo_process,
        spawn_entity_node,
        mecanum_controller_spawner
    ])