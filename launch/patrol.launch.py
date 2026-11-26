from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol_pkg',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        Node(
            package='patrol_pkg',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
            parameters=[{
                'goal_x': 1.5,
                'goal_y': 0.5
            }]
        )
    ])
