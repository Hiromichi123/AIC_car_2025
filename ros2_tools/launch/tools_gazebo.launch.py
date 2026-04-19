from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    tools_launch = os.path.join(current_dir, 'tools.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tools_launch)),
    ])
