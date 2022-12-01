from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('use_smoothing', default_value='true'),
        DeclareLaunchArgument('smoothing_factor', default_value='0.8'),
        DeclareLaunchArgument('sample_density', default_value='20.0'),

        Node(
            package='path_generators',
            executable='triangulation',
            name='path_generators_triangulation',
            output='screen',
            parameters=[
                {'use_smoothing': LaunchConfiguration('use_smoothing')},
                {'smoothing_factor': LaunchConfiguration('smoothing_factor')},
                {'sample_density': LaunchConfiguration('sample_density')},
            ],
            remappings=[
                ('/cone_map', '/ground_truth/cone_map'),
            ]
        )
    ])
