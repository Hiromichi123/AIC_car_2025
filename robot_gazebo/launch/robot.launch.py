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
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # 传递参数文件到 controller_manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": Command(['xacro ', "/home/hiro/ros2/AIC_car_2025/install/robot_gazebo/share/robot_gazebo/urdf/robot.urdf.xacro"])},
            "/home/hiro/ros2/AIC_car_2025/install/robot_gazebo/share/robot_gazebo/config/ros2_config.yaml",
        ],
        output="screen"
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
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity_node]) # 延迟5秒启动

    # 手动设置diff_drive_base_controller参数
    set_params = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.type',
                     'diff_drive_controller/DiffDriveController'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.left_wheel_names',
                     "['left_wheel2base_link']"],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.right_wheel_names',
                     "['right_wheel2base_link']"],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.wheel_separation', '0.2'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.wheel_radius', '0.0325'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.publish_rate', '50.0'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.enable_odom_tf', 'true'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.cmd_vel_timeout', '0.25'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.odom_frame_id', 'odom'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/controller_manager', 'diff_drive_base_controller.base_frame_id', 'base_footprint'],
                output='screen'
            )
        ]
    )

    # 加载并启动差速驱动控制器
    load_controller = TimerAction(
        period=35.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        xacro_urdf_process,
        robot_description,
        controller_manager,
        gazebo_process,
        delayed_spawn,
        #set_params,
        #load_controller
    ])