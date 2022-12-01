"""
This will launch the simulator and the Nav2 map server. 
This will NOT launch: the GUI, the vehicle model nor any teleoperation script.
Please use the custom.launch.py launchfile from imperial_driverless_utils to
launch everything at once.
"""

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='broadcast_transform', default_value='false', 
            description='Whether to broadcast the map->odom->base_link transform'),

        DeclareLaunchArgument(name='track_file', default_value='', 
            description='Path to the track definition file in a format supported by id_track_utils'),

        DeclareLaunchArgument('vehicle_config_file', 
            description='Path to a vehicle config.yaml file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'simulate_with_twist_interface.launch.py'
                ])
            ),
            launch_arguments=[
                ('broadcast_transform', LaunchConfiguration('broadcast_transform')),
                ('use_twist_interface', 'false'),
                ('track_file', LaunchConfiguration('track_file')),
                ('vehicle_config_file', LaunchConfiguration('vehicle_config_file'))
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
