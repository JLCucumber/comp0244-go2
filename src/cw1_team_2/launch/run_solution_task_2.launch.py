
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

    # param_file_path = os.path.join(os.getcwd(), "config", "robot_params_bug0.yaml")
    param_file_path = os.path.join(os.getcwd(), "src/cw1_team_2/cw1_team_2/config", "robot_params_bug0.yaml")
    if os.path.exists(param_file_path):
        parameter_file = ParameterFile(param_file_path, allow_substs=True)
        print("Path is correct and file exists.")
    else:
        print("Path is incorrect or file does not exist.")

    bug0_node = Node(
        package='cw1_team_2',
        executable='cw1_bug0',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            # ParameterFile(param_file_path, allow_substs=True)
        ]
    )


    waypoint_follower_node = Node(
        package='cw1_team_2',
        executable='cw1_waypoint_follower',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    waypoint_follower_group = GroupAction([
        waypoint_follower_node
    ])

    bug0_group = GroupAction([
        bug0_node
    ])


    return LaunchDescription([
        declare_use_sim_time,
        declare_gazebo_world,
        environment_setup_launch,
        waypoint_follower_group,
        bug0_group
    ])

# def generate_launch_description():
#     # Declare the use_sim_time argument
#     declare_use_sim_time = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',  # Default to true for simulation time
#         description='Use simulation (Gazebo) clock if true'
#     )

#     # Get the use_sim_time configuration to pass to nodes and included launch files
#     use_sim_time = LaunchConfiguration("use_sim_time")


#     # ros2 launch go2_config gazebo_mid360.launch.py
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("go2_config"),
#                 'launch',
#                 'gazebo_mid360.launch.py'
#             )
#         ),
#         launch_arguments={'use_sim_time': use_sim_time,
#                         # 'world_init_x': '-1.0',    # 你想要的新默认值
#                         # 'world_init_y': '-3.0',
#                         # 'world_init_z': '0.5',
#                         # 'world_init_heading': '1.57'
#         }.items()  # Pass sim_time
#     )

#     # ros2 launch fast_lio mapping.launch.py
#     mapping_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory("fast_lio"),
#             'launch',
#             'mapping.launch.py'
#             )
#         ),
#         launch_arguments={'config_file': 'unitree_go2_mid360.yaml', 'use_sim_time': use_sim_time}.items()
#     )

#     # ros2 run waypoint_follower waypoint_follower
#     waypoint_follower_node = Node(
#         package='cw1_team_2',
#         executable='cw1_waypoint_follower',
#         output='screen',
#         parameters=[{"use_sim_time": use_sim_time}]
#     )
    
#     # ros2 run local_map_creator local_map_creator
#     local_map_node = Node(
#         package='local_map_creator',
#         executable='local_map_creator',
#         output='screen',
#         parameters=[{"use_sim_time": use_sim_time}]
#     )

#     # ros2 run static_tf_node to connect the robot with the camera
#     static_tf_node = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments=['0', '0', '0', '1.57', '0', '0', 'odom', 'body'],
#         output='screen',
#         parameters=[{"use_sim_time": use_sim_time}]
#     )

#     bug0_node = Node(
#         package='cw1_team_2',
#         executable='cw1_bug0',
#         output='screen',
#         parameters=[{
#             "use_sim_time": use_sim_time
#         }]
#     )

#     # segment groups
#     gazebo_group = GroupAction([
#         gazebo_launch
#     ])
#     mapping_group = GroupAction([
#         mapping_launch
#     ])
#     local_map_group = GroupAction([
#         local_map_node
#     ])
#     static_tf_group = GroupAction([
#         static_tf_node
#     ])
#     waypoint_follower_group = GroupAction([
#         waypoint_follower_node
#     ])
#     bug0_group = GroupAction([
#         bug0_node
#     ])

#     return LaunchDescription([
#         declare_use_sim_time,  # Declare use_sim_time
#         gazebo_group,
#         mapping_group,
#         local_map_group,
#         static_tf_group,
#         waypoint_follower_group,
#         # bug0_group,
#     ])
