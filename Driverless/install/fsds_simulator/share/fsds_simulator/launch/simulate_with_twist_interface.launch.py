from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('track_file', default_value='', description='Path to a track definition file'),
        DeclareLaunchArgument('vehicle_config_file', description='Path to a vehicle config.yaml file'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'simulate_with_vcu_interface.launch.py'
                ])
            ),
            launch_arguments=[
                ('track_file', LaunchConfiguration('track_file')),
                ('vehicle_config_file', LaunchConfiguration('vehicle_config_file')),
            ]
        ),

        Node(
            name='vehicle_controller',
            package='vehicle_controllers',
            executable='basic_pid',
            parameters=[{'use_sim_time': True}],
        )
    ])
