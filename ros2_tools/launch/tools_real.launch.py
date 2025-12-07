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

        # d435 = Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     name='camera',
    #     parameters=[{
    #         'enable_color': True,
    #         'enable_depth': True,
    #         'rgb_camera.color_profile': '848x480x30',
    #         'depth_module.depth_profile': '848x480x30',
    #         'enable_infra1': True,
    #         'enable_infra2': True,
    #         'enable_sync': True, # 同步深度和彩色图像
    #         #'aligen_depth.enable': True, # 深度图对齐彩色图
    #         'rgb_camera.power_line_frequency': 2,
    #     }],
    #     output='screen'
    # )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        lidar_data_node,
        bsp_node,
        hardware_bridge_node,
    ])
