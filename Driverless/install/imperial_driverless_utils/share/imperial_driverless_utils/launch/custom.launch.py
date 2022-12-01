from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    this_launch_file_dir = str(get_package_share_path('imperial_driverless_utils') / 'launch')

    simulation_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                this_launch_file_dir,
                'setup_simulation_environment.launch.py',
            ])
        ),
    )

    software_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                this_launch_file_dir,
                'software_stack.launch.py',
            ])
        ),
    )

    return LaunchDescription([
        simulation_environment,
        software_stack,
    ])
