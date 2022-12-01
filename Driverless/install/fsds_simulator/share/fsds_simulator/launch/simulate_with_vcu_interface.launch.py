from pathlib import Path
import json
import math

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

import yaml

def rewrite_fsds_settings(vehicle_config):
    settings_path = Path('~/Formula-Student-Driverless-Simulator/settings.json').expanduser()
    settings = json.load(settings_path.open('r'))
    sensor_settings = settings['Vehicles']['FSCar']['Sensors']

    lidar_config = vehicle_config['sensors']['lidar']

    sensor_settings['Lidar1'] = {
        "SensorType": 6,
        "Enabled": True,
        "X": lidar_config['position_relative_to_base_link']['x'], 
        "Y": lidar_config['position_relative_to_base_link']['y'], 
        "Z": lidar_config['position_relative_to_base_link']['z'],
        "Roll": 0, "Pitch": 0, "Yaw" : 0,
        "NumberOfLasers": 1,
        "PointsPerScan": lidar_config['num_beams'],
        "VerticalFOVUpper": 1,
        "VerticalFOVLower": -1,
        "HorizontalFOVStart": -math.degrees(lidar_config['fov']/2),
        "HorizontalFOVEnd":    math.degrees(lidar_config['fov']/2),
        "RotationsPerSecond": 10,
        "DrawDebugPoints": False
    }
    if 'Lidar2' in sensor_settings:
        del sensor_settings['Lidar2']

    # this line is probably not needed, as python is pass-by-reference
    # but it's here just in case
    settings['Vehicles']['FSCar']['Sensors'] = sensor_settings
    # settings['Vehicles']['FSCar']['Cameras'] = {}

    json.dump(settings, settings_path.open('w'), indent=2)

    input("Rewritten FSDS settings.json file." 
          "Launch the simulator now, and press eneter when it's running."
          "If it already is running, restart it.")

def generate_ld_using_config(context, *args, **kwargs):
    vehicle_config_file = Path(LaunchConfiguration('vehicle_config_file').perform(context))

    config = yaml.safe_load(vehicle_config_file.read_text())
    lidar_config = config['sensors']['lidar']
    
    rewrite_fsds_settings(config)

    return [
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                {'min_height': -lidar_config['position_relative_to_base_link']['z'] + 0.1}, # to remove ground
                {'angle_min': -lidar_config['fov'] / 2.0},
                {'angle_max':  lidar_config['fov'] / 2.0},
                {'angle_increment': lidar_config['fov'] / lidar_config['num_beams']},
                {'range_min': 0.5},
                {'range_max': 50.0},
                # {'target_frame': 'scan'},
                {'use_sim_time': True},
            ],
            remappings=[
                ('cloud_in', '/lidar/Lidar1'),
            ]
        ),

        # this should have parameters passed from the vehicle config file
        Node(
            package='fsds_simulator',
            executable='control_command_converter',
            name='control_command_converter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('fsds_ros2_bridge'), 
                    'launch',
                    'fsds_ros2_bridge.launch.py'
                ])
            )
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('track_file', default_value='', description='Path to a track definition file'),
        DeclareLaunchArgument('vehicle_config_file', description='Path to a vehicle config.yaml file'),
        
        

        # this node assumes that we're using perfect odometry
        Node(   
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'fsds/map'],
            parameters=[{'use_sim_time': True}],
        ),

        # this node asserts that gt_map is the same as fsds/map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0', '0', '0', '0', 'fsds/map', 'gt_map']
        ),

        # this node takes ground truth odometry readings
        # and produces the fsds/map -> fsds/FSCar transform
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'two_d_mode': True,
                'publish_tf': True,
                'frequency': 50.0,
                'world_frame': 'fsds/map',
                'odom_frame': 'fsds/map',
                'base_link_frame': 'fsds/FSCar',
                'odom0': '/testing_only/odom',
                'odom0_config': [
                    True, True, True,
                    True, True, True,
                    True, False, False,
                    False, False, True,
                    False, False, False,
                ]
            }],
            remappings=[
                ('/odometry/filtered', '/odom')
            ]
        ),

        # this node asserts that fsds/FSCar frame is the same as base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['-0.6', '0', '0', '0', '0', '0', 'fsds/FSCar', 'base_link'],
            parameters=[{'use_sim_time': True}],
        ),

        # this node asserts that fsds/Lidar1 frame is the same as laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'fsds/Lidar1', 'laser'],
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='fsds_simulator',
            executable='fs_track_to_conemap',
            name='fs_track_to_conemap',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        OpaqueFunction(function=generate_ld_using_config),
    ])
