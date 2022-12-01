from typing import List
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def get_available_vehicles() -> List[str]:
    pkg_share = get_package_share_path('vehicle_descriptions')
    return [dir.name for dir in (pkg_share / 'vehicles').iterdir()]

def generate_launch_description():
    
    pkg_share = get_package_share_path('vehicle_descriptions')

    rviz_config_file = pkg_share / 'rviz' / 'urdf.rviz'

    return LaunchDescription([
        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_config_file)],
        ),

        DeclareLaunchArgument('vehicle_name', choices=get_available_vehicles()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    str(pkg_share),
                    'launch',
                    'simple.launch.py'
                ])
            ),
            launch_arguments=[
                ('vehicle_name', LaunchConfiguration('vehicle_name')),
                ('use_sim_time', 'false'),
            ]
        )
    ])
