from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cw1_team_2',
            executable='Bug1',
            name='Bug1',
            output='screen'
        )
    ])