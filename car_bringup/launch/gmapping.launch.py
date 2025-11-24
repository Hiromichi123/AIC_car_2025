from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    car_bringup_dir = get_package_share_directory('car_bringup')
    
    # Path to parameter files
    robot_localization_params = os.path.join(
        car_bringup_dir,
        'param',
        'robot_localization.yaml'
    )
    
    # Lidar launch (commented out, depends on external package)
    # lslidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('lslidar_driver'),
    #             'launch',
    #             'lslidar_serial.launch.py'
    #         ])
    #     ])
    # )
    
    # Robot model launch (commented out, depends on external package)
    # mowen_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mowen'),
    #             'launch',
    #             'display.launch.py'
    #         ])
    #     ])
    # )
    
    # IMU launch (commented out, depends on external package)
    # wit_imu_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('wit_ros_imu'),
    #             'launch',
    #             'wit_imu.launch.py'
    #         ])
    #     ])
    # )
    
    # RViz node (commented out, depends on external package)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(
    #         get_package_share_directory('nav_demo'),
    #         'config',
    #         'gmap.rviz'
    #     )]
    # )
    
    # Wheel odometry publisher
    pubv_node = Node(
        package='car_bringup',
        executable='pubv',
        name='pubv',
        output='screen'
    )
    
    # Odometry publisher
    base_node = Node(
        package='car_bringup',
        executable='base_node',
        name='odometry_publisher',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_footprint_frame': 'base_footprint',
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
        }],
        remappings=[
            ('/sub_vel', '/vel_raw'),
            ('/pub_odom', '/odom_raw'),
        ]
    )
    
    # Odometry to TF broadcaster (commented out)
    # odo_node = Node(
    #     package='car_bringup',
    #     executable='odo',
    #     name='odo',
    #     output='screen'
    # )
    
    # Extended Kalman Filter for sensor fusion
    # (Note: robot_localization package needs to be available in ROS2)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        parameters=[robot_localization_params, {
            'odom_frame': 'odom',
            'world_frame': 'odom',
            'base_link_frame': 'base_footprint',
        }],
        remappings=[
            ('odometry/filtered', 'odom'),
            ('/imu0', '/wit/imu'),
            ('/odom0', '/odom_raw'),
        ]
    )
    
    # GMapping SLAM (commented out, needs slam_gmapping ROS2 package)
    # gmapping_node = Node(
    #     package='slam_gmapping',
    #     executable='slam_gmapping',
    #     name='slam_gmapping',
    #     output='screen',
    #     parameters=[{
    #         'map_update_interval': 1.0,
    #         'maxUrange': 6.0,
    #         'sigma': 0.05,
    #         'kernelSize': 1.0,
    #         'lstep': 0.05,
    #         'astep': 0.05,
    #         'iterations': 5.0,
    #         'lsigma': 0.075,
    #         'ogain': 3.0,
    #         'lskip': 0.0,
    #         'minimumScore': 4.0,
    #         'srr': 0.1,
    #         'srt': 0.2,
    #         'str': 0.1,
    #         'stt': 0.2,
    #         'linearUpdate': 0.04,
    #         'angularUpdate': 0.05,
    #         'temporalUpdate': -1.0,
    #         'resampleThreshold': 0.5,
    #         'particles': 150,
    #         'xmin': -7.0,
    #         'ymin': -1.0,
    #         'xmax': 1.0,
    #         'ymax': 7.0,
    #         'delta': 0.01,
    #         'llsamplerange': 0.01,
    #         'llsamplestep': 0.01,
    #         'lasamplerange': 0.005,
    #         'lasamplestep': 0.005,
    #     }]
    # )
    
    return LaunchDescription([
        # lslidar_launch,
        # mowen_launch,
        # wit_imu_launch,
        # rviz_node,
        pubv_node,
        base_node,
        # odo_node,
        ekf_node,
        # gmapping_node,
    ])
