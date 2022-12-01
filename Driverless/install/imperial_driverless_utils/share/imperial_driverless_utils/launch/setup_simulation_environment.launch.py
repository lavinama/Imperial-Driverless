from pathlib import Path
from tempfile import mkdtemp
from typing import Any, Dict, List, Literal, Optional, Tuple, Union

import time

import click
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource

from ament_index_python.packages import PackageNotFoundError, get_package_share_path

from id_track_utils.track import Track # has to be apt-installed to be detected

from dataclasses import dataclass

RvizDisplay = Dict[Any, Any]

# this definition ensures that we don't make typos in simulator names
SimulatorName = Union[
    Literal['lightweight_lidar_only_simulator'],
    Literal['full_gazebo_simulator'],
    Literal['fsds_simulator'],
]

# click colours for terminal output
def red(x: str, /)   : return click.style(x, fg='red')
def blue(x: str, /)  : return click.style(x, fg='blue')
def green(x: str, /) : return click.style(x, fg='green')
def cyan(x: str, /)  : return click.style(x, fg='cyan')
def yellow(x: str, /): return click.style(x, fg='yellow')

LaunchOption = Tuple[str, List[LaunchDescriptionEntity]]

@dataclass(frozen=True)
class Simulator:
    name: SimulatorName

    @property
    def launch_files(self) -> List[Path]:
        pkg_path = get_package_share_path(self.name)
        return list((pkg_path / 'launch').glob('*.launch*'))

supported_simulators: List[Simulator] = [
    Simulator('lightweight_lidar_only_simulator'), 
    Simulator('full_gazebo_simulator'),
    Simulator('fsds_simulator'),
]

def generate_launch_description():
    tmp_dir = Path(mkdtemp())

    click.clear()
    try:
        vehicle_ld, vehicle_config_file = choose_vehicle()

        sim, sim_launch_file = select_simulator_and_launchfile()

        if sim.name == 'fsds_simulator':
            track_file = None
            ground_truth_cone_map_ld = []
        else:
            track_file = select_racetrack_map(tmp_dir)
            ground_truth_cone_map_ld = ground_truth_cone_map_publisher(track_file)

        sim_ld = generate_simulator_ld(
            sim, 
            sim_launch_file, 
            vehicle_config_file,
            track_file, 
        )

        clock_sync_signal_emitter = Node(
            package='imperial_driverless_utils',
            executable='clock_sync_signal_emitter',
            name='clock_sync_signal_emitter',
            parameters=[{'use_sim_time': True}],
        )

        # odometry_noisifier = Node(
        #     package="imperial_driverless_utils",
        #     executable="odometry_noisifier",
        #     name="odometry_noisifier",
        #     output="screen",
        #     parameters=[
        #         {'use_sim_time': True},
        #         {"drift_magnitude": 0.1},
        #     ],
        #     remappings=[("/odom", "/odom_noisy")],
        # )

        return LaunchDescription([
            clock_sync_signal_emitter,
            *ground_truth_cone_map_ld,
            *sim_ld,
            *vehicle_ld,
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

def ground_truth_cone_map_publisher(track_file: Path) -> List[LaunchDescriptionEntity]:
    gt_cone_map_publisher = Node(
            package='imperial_driverless_utils',
            executable='gt_cone_map_publisher',
            name='gt_cone_map_publisher',
            parameters=[{'track_file': str(track_file),            
                'use_sim_time': True
            }],
    )


    return [
        gt_cone_map_publisher,
    ]

def select_simulator_and_launchfile() -> Tuple[Simulator, Path]:
    launch_files: Dict[Simulator, List[Path]] = {sim: [] for sim in supported_simulators}

    for sim, sim_launch_files in launch_files.items():
        try:
            sim_launch_files.extend(sim.launch_files)
        except PackageNotFoundError:
            click.echo(yellow(f'{sim.name} not found'))
            

    if not any(launch_files):
        click.echo(red(f'No supported simulator launchfiles found. Supported simulators: {supported_simulators}'))
        raise click.Abort()

    click.echo(f'Select a simulator launch file:')
    
    all_launch_files = [
        (sim_name, launch_file.stem, launch_file)
        for sim_name, sim_launch_files in launch_files.items()
        for launch_file in sim_launch_files
    ]

    for i, (sim, launch_file_stem, _) in enumerate(all_launch_files):
        click.echo(f'{i}. {blue(sim.name)} {green(launch_file_stem)}')

    selection = click.prompt(
        'Please select:',
        type=click.Choice([str(i) for i, _ in enumerate(all_launch_files)]),
        default='0',
        show_choices=False,
    )

    assert isinstance(selection, str)
    selected_sim, _, selected_launch_file = all_launch_files[int(selection)]
    
    click.echo(f'Launching {green(selected_launch_file.stem)} '
               f'from {blue(selected_sim.name)}.\n')

    return selected_sim, selected_launch_file

def generate_simulator_ld(
    sim: Simulator,
    simulator_launch_file: Path,
    vehicle_config_file: Path,
    track_file: Optional[Path],
) -> List[LaunchDescriptionEntity]:

    include_simulator_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulator_launch_file)
        ),
        launch_arguments=[
            ('track_file', str(track_file) if track_file else ''),
            ('vehicle_config_file', str(vehicle_config_file)),
        ]
    )

    return [include_simulator_ld]

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
    config_yaml_path = vehicle_descriptions_share / 'vehicles' / vehicle_name / 'config.yaml'
    click.echo('Using vehicle ' + green(vehicle_name) + '.\n')
    
    include_vehicle_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(vehicle_descriptions_share / 'launch' / 'simple.launch.py')
        ),
        launch_arguments=[
            ('vehicle_name', vehicle_name)
        ]
    )

    return [include_vehicle_ld], config_yaml_path
