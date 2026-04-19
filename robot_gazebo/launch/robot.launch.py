from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, Shutdown, DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(pkg_path, '..', 'worlds', 'empty.world')
    urdf_xacro = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf.xacro')
    urdf_file = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf')
    yaml_file = os.path.join(pkg_path, '..', 'urdf', 'mecanum_controllers.yaml')
    gazebo_master_uri = os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11346')
    gui = LaunchConfiguration('gui')

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo client GUI (gzclient).'
    )

    set_gazebo_master_uri = SetEnvironmentVariable(
        name='GAZEBO_MASTER_URI',
        value=gazebo_master_uri,
    )

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
        cmd=['gzserver',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            world_file,
        ],
        output='screen',
        on_exit=Shutdown(reason='Gazebo server exited')
    )

    gazebo_client_process = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )
    gazebo_client_process = TimerAction(period=2.0, actions=[gazebo_client_process])

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
            '--param', '/home/dev/ros_ws/rs_ws/src/AIC_car_2025/robot_gazebo/urdf/mecanum_controllers.yaml'
        ],
        output='screen'
    )
    mecanum_controller_spawner = TimerAction(period=10.0, actions=[mecanum_controller_spawner]) # 延迟10秒启动

    return LaunchDescription([
        declare_gui,
        set_gazebo_master_uri,
        robot_description,
        gazebo_process,
        gazebo_client_process,
        spawn_entity_node,
    ])