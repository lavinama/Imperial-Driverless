from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import   IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_simulator_with_vcu_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('full_gazebo_simulator'),
                'launch',
                'simulate_with_vcu_interface.launch.py'
            ])
        ),
        launch_arguments=[
            ('track_file', LaunchConfiguration('track_file')),
            ('vehicle_config_file', LaunchConfiguration('vehicle_config_file')),
        ]
    )

    twist_to_vcu_vehicle_controller = Node(
        name='vehicle_controller',
        package='vehicle_controllers',
        executable='basic_pid',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('track_file', default_value='', description='Path to a track definition file'),
        DeclareLaunchArgument('vehicle_config_file', description='Path to a vehicle config.yaml file'),
        launch_simulator_with_vcu_interface,
        twist_to_vcu_vehicle_controller,
    ])
