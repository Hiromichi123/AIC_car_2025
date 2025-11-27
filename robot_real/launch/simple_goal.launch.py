from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    goal_parameters = {
        'goal_points': [
            '0.0,0.0,0.0',
            '1.0,0.0,0.0',
            '1.0,1.0,1.57',
        ],
        'publish_interval': 3.0,
        'frame_id': 'odom',
        'loop': False,
    }

    goal_node = Node(
        package='robot_real',
        executable='robot_real',
        name='simple_goal_publisher',
        output='screen',
        parameters=[goal_parameters],
    )

    return LaunchDescription([goal_node])
