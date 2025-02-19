from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cw1_team_2',
            executable='obstacle_follower',
            name='obstacle_follower',
            output='screen'
        )
    ])


