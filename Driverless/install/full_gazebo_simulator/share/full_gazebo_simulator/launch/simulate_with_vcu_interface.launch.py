from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import   IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from id_track_utils import Track
import xml.etree.ElementTree

import psutil

from ament_index_python.packages import get_package_share_path

from tempfile import mkdtemp
from pathlib import Path

def generate_gazebo_world(context, tmp_dir, *args, **kwargs):
    track_file = LaunchConfiguration('track_file').perform(context)

    empty_world = get_package_share_path('full_gazebo_simulator') / 'worlds' / 'empty.world'
    doc = xml.etree.ElementTree.parse(empty_world)

    if track_file:
        track: Track = Track.load_from_file(track_file)

        
        world = doc.getroot().find('world')
        assert world is not None
        
        for e in track.as_gazebo_sdf('some_name').iter('include'):
            static_tag = xml.etree.ElementTree.Element('static')
            static_tag.text = '1' 
            allow_auto_disable_tag = xml.etree.ElementTree.Element('allow_auto_disable')
            allow_auto_disable_tag.text = '1' 
            e.append(static_tag)
            e.append(allow_auto_disable_tag)
            world.append(e)
    else:
        print('WARNING: No track file specified, using empty world')

    doc.write(Path(tmp_dir, 'world.world'), encoding='utf-8', xml_declaration=False)

def spawn_vehicle(context):
    track_file = LaunchConfiguration('track_file').perform(context)
    track = Track.load_from_file(track_file)
    return [
        Node(
            name='gazebo_simulation_id_interface',
            package='full_gazebo_simulator',
            executable='id_interface',
            parameters=[
                {'urdf_include': '$(find full_gazebo_simulator)/urdf_extensions/gazebo_extension.xacro'},
                {'vehicle_config_path': LaunchConfiguration('vehicle_config_file')},
                {'x': track.start_x},
                {'y': track.start_y},
                {'th': track.start_yaw}
            ]
        )
    ]

def generate_launch_description():
    tmp_dir = mkdtemp()

    full_gazebo_simulator_pkg_share = get_package_share_path('full_gazebo_simulator')

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch/gzserver.launch.py'
            ])
        ),
        launch_arguments=[
            ('world', PathJoinSubstitution([tmp_dir, 'world.world'])),
            ('required', 'true'),
        ],
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        str(full_gazebo_simulator_pkg_share / 'models')
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        ':'.join([
            str(full_gazebo_simulator_pkg_share / 'meshes'),
            '/usr/share/gazebo-11'
        ])
    )

    # kill gazebo if it is already running
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            if 'gzserver' in proc.name():
                proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    

    return LaunchDescription([
        DeclareLaunchArgument('track_file', default_value='', description='Path to a track definition file'),
        DeclareLaunchArgument('vehicle_config_file', description='Path to a vehicle config.yaml file'),
        set_gazebo_model_path,
        set_gazebo_resource_path,
        OpaqueFunction(function=generate_gazebo_world, args=[tmp_dir]),
        launch_gazebo,
        OpaqueFunction(function=spawn_vehicle),
    ])
