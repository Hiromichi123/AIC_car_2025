from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(pkg_path, '..', 'worlds', 'empty.world')
    urdf_xacro = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf.xacro')
    urdf_file = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf')
    yaml_file = os.path.join(pkg_path, '..', 'config', 'ros2_config.yaml')

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
        #parameters=[{'robot_description': Command(['xacro', urdf_file, 'ros2_control_yaml:=', yaml_file])}]
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # 启动 Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', 
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            world_file],
        output='screen'
    )

    # 使用参数服务器加载URDF
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',  # 从参数服务器获取URDF
            '-entity', 'mycar',
            '-x', '1.65',
            '-y', '1.65',
            '-z', '0.05'
        ],
        output='screen'
    )

    return LaunchDescription([
        xacro_urdf_process,
        robot_description,
        gazebo_process,
        spawn_entity_node
    ])