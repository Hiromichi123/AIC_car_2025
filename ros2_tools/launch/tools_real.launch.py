from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyCH341USB0',
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
    )

    # LidarDataNode 雷达数据处理节点
    lidar_data_node = Node(
        package='ros2_tools',
        executable='lidar_data_node',
        name='lidar_data_node',
        output='screen',
        parameters=[{
            'use_simulation': False, # 是否使用仿真模式
            'simulation_odom_topic': '/absolute_pose', # 仅在仿真中使用
            'real_robot_odom_topic': '/aft_mapped_to_init', # 实际机器人里使用
        }]
    )

    # BSP 控制器（中间层pid）
    bsp_node = Node(
        package='ros2_tools',
        executable='bsp_node',
        name='bsp_node',
        output='screen'
    )

    # 硬件桥接节点 - 串口通信
    hardware_bridge_node = Node(
        package='ros2_tools',
        executable='hardware_bridge_node',
        name='hardware_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'max_linear_speed': 1.5,
            'max_angular_speed': 1.0
        }]
    )

# D435相机节点
    d435_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'rgb_camera.color_profile': '1920x1080x30',
            'enable_infra1': False, # 红外
            'enable_infra2': False,
            #'enable_sync': True, # 同步深度和彩色图像
            #'aligen_depth.enable': True, # 深度图对齐彩色图
            'rgb_camera.power_line_frequency': 1, # 50Hz供电频率
        }],
        output='screen'
    )

    # 单目相机节点
    camera_node = Node(
        package='ros2_tools',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    # 舵机控制节点
    pwm_node = Node(
        package='ros2_tools',
        executable='pwm_node',
        name='pwm_node',
        output='screen',
    )

    # tts节点
    tts_node = Node(
        package='tts',
        executable='tts_node',
        name='tts_node',
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        lidar_data_node,
        bsp_node,
        hardware_bridge_node,
        d435_node,
        camera_node,
        pwm_node,
        tts_node,
    ])
