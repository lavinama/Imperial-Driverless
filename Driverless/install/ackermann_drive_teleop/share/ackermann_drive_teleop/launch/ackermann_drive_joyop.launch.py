import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='max_speed',
            default_value='0.2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_angle',
            default_value='0.7'
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy'
        ),
        launch_ros.actions.Node(
            package='ackermann_drive_teleop',
            executable='ackermann_drive_joyop.py',
            name='ackermann_drive_joyop',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
