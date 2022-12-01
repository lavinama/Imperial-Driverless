import os
from pathlib import Path
from tempfile import mkdtemp
from typing import Any, Dict, Iterable, List, Tuple, TypeVar
import enum

import time

import click
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource
from ros2pkg.api import get_executable_paths, PackageNotFound

from ament_index_python.packages import PackageNotFoundError, get_package_share_path

from imperial_driverless_utils.rviz_config import (
    get_rviz_config, marker_array_display, cone_map_display, path_display, RvizDisplay)

from id_track_utils.track import Track # has to be apt-installed to be detected

from dataclasses import dataclass, field



# click colours for terminal output
def red(x: str, /)   : return click.style(x, fg='red')
def blue(x: str, /)  : return click.style(x, fg='blue')
def green(x: str, /) : return click.style(x, fg='green')
def cyan(x: str, /)  : return click.style(x, fg='cyan')
def yellow(x: str, /): return click.style(x, fg='yellow')

# possible sources of a package
class PackageSource(enum.Enum): 
    NOT_FOUND = 'package was not found',
    SYSTEM    = 'package was installed globaly (probably via apt)'
    LOCAL     = 'package was installed locally'


LaunchOption = Tuple[str, List[LaunchDescriptionEntity]]

@dataclass
class Package:
    '''
    This class represents a ROS2 package.
    '''
    name: str
    url: str

    rviz_displays: List[RvizDisplay]
    custom_options: List[LaunchOption] = field(default_factory=list)
    fallback_options: List[LaunchOption] = field(default_factory=list)

    @property
    def executable_names(self) -> List[str]:
        return [os.path.basename(path) for path in get_executable_paths(package_name=self.name)] # type: ignore

    @property
    def launch_files(self) -> Iterable[Path]:
        pkg_path = get_package_share_path(self.name)
        return (pkg_path / 'launch').glob('*.launch*')

    @property
    def source(self) -> PackageSource:
        try:
            if get_package_share_path(self.name).parts[1] == 'opt':
                return PackageSource.SYSTEM
            else:
                return PackageSource.LOCAL
        except PackageNotFoundError:
            return PackageSource.NOT_FOUND

collection_pkgs = [
    Package('lidar_only_cone_detector', 
        url='https://github.com/Imperial-Driverless/lidar_only_cone_detector',
        rviz_displays=[cone_map_display('/perceived_cones/lidar', 'Lidar Perceived Cones')]
    ),
    Package('camera',
        url='https://github.com/Imperial-Driverless/camera',
        rviz_displays=[cone_map_display('/perceived_cones/camera', 'Camera Perceived Cones')],
    ),
    Package('lidar_camera_fusion',
        url='https://github.com/Imperial-Driverless/lidar_camera_fusion',
        rviz_displays=[cone_map_display('/perceived_cones/fused', 'Fused Perceived Cones')],
    ),
    Package('slam_implementations',
        url='https://github.com/Imperial-Driverless/slam_implementations',
        rviz_displays=[cone_map_display('/cone_map', 'SLAM Cone Map')],
        fallback_options=[
            (cyan('publish a static identity transform from map to odom'), [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_odom_static_tf_publisher',
                    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
                )
            ])
        ]
    ),
    Package('path_generators',
        url='https://github.com/Imperial-Driverless/path_generators',
        rviz_displays=[
            path_display('/path', 'Generated Path', (25, 255, 0)),
            marker_array_display('/visualization/path_generator', 'Path Generator Visualization'),
        ],
    ),
    Package('path_followers',
        url='https://github.com/Imperial-Driverless/path_followers',
        rviz_displays=[],
        fallback_options=[
            (click.style('launch twist from car_keyop', fg='cyan'), [
                Node(
                    package='car_keyop',
                    executable='twist',
                    name='car_keyop',
                    arguments=['8.0', '1'], # max linear and angular velocities
                    parameters=[{'use_sim_time': True}],
                )
            ])
        ]
    ),
]

T = TypeVar('T')
def flatten_dict_of_lists_to_list(d: Dict[Any, List[T]], /) -> List[T]:
    return [x for xs in d.values() for x in xs]


def generate_launch_description():
    tmp_dir = Path(mkdtemp())

    rviz_displays: Dict[str, List[RvizDisplay]] = {}

    click.clear()
    try:
        package_nodes, rviz_displays['from nodes'] = package_nodes_ld()


        path_from_odom_history_publisher = Node(
            package='imperial_driverless_utils',
            executable='path_from_odom_history_publisher',
            name='path_from_odom_history_publisher',
            parameters=[{'use_sim_time': True},
                        {'history_length_seconds': 20}]
        )
        rviz_displays['path_history'] = [
            path_display('/gt_odom_path', 'Ground truth odom frame path history', (0, 255, 0)),
            path_display('/loc_odom_path', 'Robot localization odom frame path history', (255, 255, 0))
        ] 
        rviz_displays['ground truth cone map visualization'] = [cone_map_display('/ground_truth/cone_map', 'Ground Truth Cone Map')]

        all_rviz_displays = flatten_dict_of_lists_to_list(rviz_displays)

        return LaunchDescription([
            path_from_odom_history_publisher,
            *rviz_ld(tmp_dir, all_rviz_displays),
            *package_nodes,
        ])

    except click.Abort:
        click.echo()
        click.echo(yellow('Launch aborted by the user'))
        exit()
      
def select_racetrack_map(tmp_dir: Path) -> Path:
    cone_map_folder = get_package_share_path('imperial_driverless_utils') / 'cone_maps'

    cone_map_paths = [x for x in cone_map_folder.iterdir()]

    click.echo(f'Select a cone map:')
    for i, cone_map in enumerate(cone_map_paths):
        click.echo( f'{i}. {green(cone_map.name)}')
    click.echo(f"R. {cyan('random track')}")

    selection = click.prompt(
        'Please select:',
        type=click.Choice([str(i) for i in range(len(cone_map_paths))] + ['R', 'r']),
        default='0',
        show_choices=False,
    )

    assert isinstance(selection, str)
    if selection in ['r', 'R']:
        default_seed = int(time.time()) % 1000
        seed = click.prompt('(optional) random seed:', default=default_seed)
        t = Track.generate_random(seed)
        selected_track_path = tmp_dir / 'track.csv'
        selected_track_path.write_text(t.as_csv())
        click.echo(f'Using a random track with seed {seed}.\n')
    else:
        selected_track_path = cone_map_paths[int(selection)]
        click.echo(f'Using track {green(selected_track_path.name)}.\n')

    return selected_track_path

def rviz_ld(tmp_dir: Path, displays: List[RvizDisplay]) -> List[LaunchDescriptionEntity]:
    rviz_config_file = tmp_dir / 'rviz_config.rviz'
    rviz_config_file.write_text(get_rviz_config(displays))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-d', str(rviz_config_file),
            '--ros-args', '--log-level', 'error',
        ])

    shutdown_on_rviz_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                EmitEvent(event=Shutdown()),
            ],
        ))
    
    return [rviz_node, shutdown_on_rviz_exit]

def package_nodes_ld() -> Tuple[List[LaunchDescriptionEntity], List[RvizDisplay]]:
    pkg_found_common_options: List[LaunchOption] = [
        (yellow('do not launch any executable from this package'), [])
    ]

    pkg_not_found_common_options: List[LaunchOption] = [
        (yellow('ignore, I will start it myself'), []),
        # (click.style('Clone from GitHub, build and source', fg='yellow'), []),
    ]

    ld: List[LaunchDescriptionEntity] = []
    rviz_displays: List[RvizDisplay] = []

    for pkg in collection_pkgs:
        all_options = []
        launch_file_options = []
        executable_options = []

        try:
            executable_options: List[LaunchOption] = [(
                green(executable), [
                Node(
                    package=pkg.name,
                    executable=executable,
                    name=executable,
                    emulate_tty=True,
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                )
            ]) for executable in pkg.executable_names]

            launch_file_options: List[LaunchOption] = [(
                cyan(launch_file.stem), [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(launch_file)),
                )
            ]) for launch_file in pkg.launch_files]
            
            all_options: List[LaunchOption] = [
                *pkg.fallback_options,
                *pkg_found_common_options,
                *executable_options,
                *launch_file_options,
                *pkg.custom_options,
            ]

            click.echo(f'Select an executable from {blue(pkg.name)} [{pkg.source.name}]')


        except PackageNotFound:
            click.echo(blue(pkg.name) + red(' not found. What to do?'))
            
            all_options = [
                *pkg.fallback_options,
                *pkg_not_found_common_options,
            ]

            
        for i, (text, _) in enumerate(all_options):
            click.echo( f'{i}. {text}')
        
        selection = click.prompt(
            'Please select:',
            type=click.Choice([str(i) for i, _ in enumerate(all_options)]),
            default='0',
            show_choices=False,
        )

        assert isinstance(selection, str)
        selected_option = all_options[int(selection)]
        _, entities = selected_option

        if selected_option in executable_options or selected_option in launch_file_options:
            click.echo('Launching ' + selected_option[0] + \
            f' from {blue(pkg.name)}.\n')
        else:
            click.echo()
        
        ld.extend(entities)

        # if we're launching something we might need its visualization.
        if entities:
            rviz_displays.extend(pkg.rviz_displays)

    return ld, rviz_displays

def choose_vehicle() -> Tuple[List[LaunchDescriptionEntity], Path]:
    
    vehicle_descriptions_share = get_package_share_path('vehicle_descriptions')
    
    vehicle_names = [p.name for p in vehicle_descriptions_share.glob('vehicles/*')]

    click.echo('Select a vehicle')

    for i, vehicle_name in enumerate(vehicle_names):
        click.echo( f'{i}. {green(vehicle_name)}')

    selection = click.prompt(
        'Please select:',
        type=click.Choice([str(i) for i, _ in enumerate(vehicle_names)]),
        default='0',
        show_choices=False,
    )


    assert isinstance(selection, str)
    vehicle_name = vehicle_names[int(selection)]

    click.echo('Using vehicle ' + green(vehicle_name) + '.\n')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(vehicle_descriptions_share / 'launch' / 'simple.launch.py')
            ),
            launch_arguments=[
                ('vehicle_name', vehicle_name)
            ]
        )
    ], vehicle_descriptions_share / 'vehicles' / vehicle_name / 'config.yaml'
