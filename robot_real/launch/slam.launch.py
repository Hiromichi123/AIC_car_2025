from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    livox_ros_driver2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"),
                "launch_ROS2",
                "msg_MID360_launch.py"
            ])
        ])
    )

    # 发布 base_link 到 livox_frame 的静态TF
    tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox_tf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "livox_frame"]
    )

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("point_lio"),
                "launch",
                "point_lio.launch.py"
            ])
        ]),
        launch_arguments={"rviz": "False"}.items(),
    )

    return LaunchDescription([
        livox_ros_driver2,
        tf_pub,
        point_lio,
    ])