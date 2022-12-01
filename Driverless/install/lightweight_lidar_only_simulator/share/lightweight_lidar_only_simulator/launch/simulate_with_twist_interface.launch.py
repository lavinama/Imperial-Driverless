"""
This will launch the simulator and the Nav2 map server. 
This will NOT launch: the GUI, the vehicle model nor any teleoperation script.
Please use the custom.launch.py launchfile from imperial_driverless_utils to
launch everything at once.
"""

from datetime import timedelta
from pathlib import Path
from tempfile import mkdtemp

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from id_track_utils import Track
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration

import yaml


def delayed(e: EmitEvent, by: timedelta) -> ExecuteProcess:
    return ExecuteProcess(cmd=["sleep", str(by.total_seconds())], on_exit=[e])


def launch_map_server(context, *args, **kwargs):
    track_file = LaunchConfiguration("track_file").perform(context)
    map_yaml_file = LaunchConfiguration("map_yaml_file").perform(context)

    # if no yaml file passed, use the track file
    if not map_yaml_file:
        if not track_file:
            raise Exception(
                "No track file specified. Use `ros2 launch lighweight_lidar_only_simulator "
                "simulate.launch.py track_file:=<path to track file>` to specify one."
            )

        t = Track.load_from_file(track_file)
        tmp_dir = Path(mkdtemp())

        resolution = 0.03
        map_image, orig_x, orig_y = t.as_occupancy_grid(
            resolution=resolution, cone_radius=0.10
        )

        map_image.save(tmp_dir / "map.png")
        map_yaml_file = tmp_dir / "map.yaml"
        map_yaml_file.write_text(
            f"""
            image: map.png
            resolution: {resolution}
            origin:  [{orig_x}, {orig_y}, 0.]
            negate: 0
            occupied_thresh: 0.8
            free_thresh: 0.2
            """
        )

    # ================
    # == MAP SERVER ==
    # ================
    map_server_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        namespace="",
        parameters=[
            {"yaml_filename": str(map_yaml_file)},
            {"topic_name": "/ground_truth/map"},
        ],
    )

    map_server_emit_activation_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    map_server_emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    map_server_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state="inactive",
            entities=[map_server_emit_activation_event],
        )
    )

    return [
        map_server_node,
        map_server_inactive_state_handler,
        delayed(map_server_emit_configure_event, timedelta(seconds=2.0)),
    ]


def launch_simulator(context, *args, **kwargs):
    vehicle_config_file = Path(
        LaunchConfiguration("vehicle_config_file").perform(context)
    )

    broadcast_transform = (
        LaunchConfiguration("broadcast_transform").perform(context) == "true"
    )
    remappings = [("/ground_truth/odom", "/odom")] if broadcast_transform else []

    track_file = LaunchConfiguration("track_file").perform(context)
    t = Track.load_from_file(track_file)

    config = yaml.safe_load(vehicle_config_file.read_text())
    lidar_config = config["sensors"]["lidar"]

    return [
        Node(
            package="lightweight_lidar_only_simulator",
            executable="simulate",
            name="lightweight_lidar_only_simulator",
            output="screen",
            parameters=[
                {"use_twist_interface": LaunchConfiguration("use_twist_interface")},
                {"update_pose_frequency": 100.0},
                {"vcu_feedback_frequency": 50.0},
                {"odometry_publishing_frequency": 100.0},
                {"scan_beams": lidar_config["num_beams"]},
                {"scan_field_of_view": float(lidar_config["fov"])},
                {"scan_frequency": float(lidar_config["update_rate"])},
                {"scan_std_dev": float(lidar_config["noise_std_dev"])},
                {"map_free_threshold": 0.3},
                {"max_speed": 10.0},  # meters/second
                {"max_steering_angle": 0.4},  # radians
                {"vehicle_mass": float(config["chassis"]["mass"])},
                {"max_torque": 195.0},  # Nm
                {
                    "max_braking_torque": 195.0
                },  # Nm, currently not being used (although this comment might be outdated)
                {
                    "broadcast_odom_base_link_transform": LaunchConfiguration(
                        "broadcast_transform"
                    )
                },
                {"broadcast_map_odom_transform": False},
                {"broadcast_base_link_gt_map_transform": True},
                {"map_frame": "map"},
                {"odom_frame": "odom"},
                {"base_frame": "base_link"},
                {"scan_frame": "laser"},
                {"initial_x": t.start_x},
                {"initial_y": t.start_y},
            ],
            remappings=remappings,
        )
    ]

def launch_robot_localization(context, *args, **kwargs):
    track_file = LaunchConfiguration("track_file").perform(context)
    t = Track.load_from_file(track_file)
    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node",
            output="screen",
            parameters=[
                {
                    "two_d_mode": True,
                    "use_sim_time": True,
                    "smooth_lagged_data": True,
                    "history_length": 1.0,
                    "reset_on_time_jump": True,
                    "publish_tf": True,
                    "map_frame": "map",
                    "odom_frame": "odom",
                    "base_link_frame": "base_link",
                    "world_frame": "odom",
                    "twist0": "/odom/vcu_drive_feedback", # contains linear velocity and angular velocity
                    "twist0_relative": True,
                    "twist0_differential": True,
                    "twist0_config": [
                        False, False, False,
                        False, False, False,
                        True,  True,  False,
                        False, False,  True,
                        False, False, False,
                    ],
                    "imu0": "/imu/data", # contains angular velocity and linear acceleration
                    "imu0_relative": True,
                    "imu0_differential": False,
                    "imu0_config": [
                        False, False, False,
                        False, False, False,
                        False, False, False,
                        False,  False,  True,
                        True,  False,  False,
                    ],
                    "initial_state": [
                        t.start_x, t.start_y, 0,
                        0,       0,       0,
                        0,       0,       0,
                        0,       0,       0,
                        0,       0,       0,
                    ],
                }
            ],
        )
    ]

def launch_vcu_processor(context, *args, **kwargs):
    # ros2 run vcu_feedback_processors simple
    # can be refactored to the custom launcher when other simulators support VCU
    return [
        Node(
            package="vcu_feedback_processors",
            executable="simple",
            name="vcu_feedback_processors",
            output="screen",
        )
    ]
    

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="broadcast_transform",
                default_value="false",
                description="Whether to broadcast the odom->base_link transform",
            ),
            DeclareLaunchArgument(
                name="use_twist_interface",
                default_value="true",
                description="If false the vcu interface will be used",
            ),
            DeclareLaunchArgument(
                name="track_file",
                default_value="",
                description="Path to the track definition file in a format supported by id_track_utils",
            ),
            DeclareLaunchArgument(
                name="map_yaml_file",
                default_value="",
                description="Path to the yaml file of the modified occupancy grid map",
            ),
            DeclareLaunchArgument(
                "vehicle_config_file", description="Path to a vehicle config.yaml file"
            ),
            OpaqueFunction(function=launch_map_server),
            OpaqueFunction(function=launch_simulator),
            OpaqueFunction(function=launch_robot_localization),
            OpaqueFunction(function=launch_vcu_processor),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
