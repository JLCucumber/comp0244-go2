
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare the use_sim_time argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Default to true for simulation time
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gazebo_world = DeclareLaunchArgument(
        'gazebo_world',
        default_value='world_4',
        description='Path to the Gazebo world file'
    )

    # Get the use_sim_time configuration to pass to nodes and included launch files
    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo_world = LaunchConfiguration("gazebo_world")

    # Include environment_setup.launch.py
    environment_setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cw1_team_2'),
                'launch',
                'environment_setup.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gazebo_world': gazebo_world
            }.items()
    )

    # param_file_path = os.path.join(os.getcwd(), "config", "robot_params_bug1.yaml")
    param_file_path = os.path.join(os.getcwd(), "src/cw1_team_2/cw1_team_2/config", "robot_params_bug1.yaml")
    if os.path.exists(param_file_path):
        parameter_file = ParameterFile(param_file_path, allow_substs=True)
        print("Path is correct and file exists.")
    else:
        print("Path is incorrect or file does not exist.")

    bug1_node = Node(
        package='cw1_team_2',
        executable='cw1_bug1',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            ParameterFile(param_file_path, allow_substs=True)
        ]
    )

    bug1_group = GroupAction([
        bug1_node
    ])


    return LaunchDescription([
        declare_use_sim_time,
        declare_gazebo_world,
        environment_setup_launch,
        bug1_group
    ])