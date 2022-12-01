from typing import List

import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_path

def create_vehicle(context, *args, **kwargs):
    pkg_share = get_package_share_path('vehicle_descriptions')

    xacro_path = pkg_share / 'urdf' / 'main.urdf.xacro'

    robot_description = xacro.process(str(xacro_path),
        mappings={
            'vehicle_name': LaunchConfiguration('vehicle_name').perform(context),
        }
    )

    use_sim_time = 'true' == LaunchConfiguration('use_sim_time').perform(context)

    return [
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'rate': 25,                
                'ignore_timestamp': True,
                'use_sim_time': use_sim_time,
            }],
            arguments=['--ros-args', '--log-level', 'warn']
        ),

        Node(
            name='joint_state_publisher',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'rate': 25,
                # 'use_sim_time': use_sim_time,
                'source_list': ['/ground_truth/joint_states'],
            }],
        ),
    ]

def get_available_vehicles() -> List[str]:
    pkg_share = get_package_share_path('vehicle_descriptions')
    return [dir.name for dir in (pkg_share / 'vehicles').iterdir()]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_name', choices=get_available_vehicles()),
        DeclareLaunchArgument('use_sim_time', choices=['true', 'false'], default_value='true'),
        OpaqueFunction(function=create_vehicle)
    ])
