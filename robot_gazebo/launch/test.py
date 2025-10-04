from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = os.path.dirname(__file__)
    urdf_xacro = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf.xacro')
    yaml_file = os.path.join(pkg_path, '..', 'urdf', 'mecanum_controllers.yaml')

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {'robot_description': Command(['xacro ', urdf_xacro])},
                {"ros2_control_config": yaml_file}
            ],
            output="screen",
        )
    ])
