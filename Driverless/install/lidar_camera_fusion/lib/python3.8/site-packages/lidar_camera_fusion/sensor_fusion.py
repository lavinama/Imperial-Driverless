from enum import Enum
import itertools
import sys
from typing import Dict, Optional, Protocol, Tuple, List

import rclpy
import rclpy.node
import rclpy.qos

import rtree  # type: ignore

import imperial_driverless_interfaces.msg as idi
import tf2_ros
import rclpy.parameter
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.qos import qos_profile_sensor_data
import rclpy.time
import rclpy.duration


from imperial_driverless_utils.tf_message_filter import TFMessageFilter


class FusionColorLookup(rclpy.node.Node):
    """
    This class implements a sensor fusion algorithm, which is supposed to work as follows:
    1. Receive the perceived cones from the camera
    2. Add them to a spatial buffer supporting k-nearest-neighbor queries,
       and remove from that buffer readings that are too old.
    3. Receive the perceived cones from the lidar
    4. Assign cone types to the lidar perceived cones by considering
       the nearby camera cone readings in the spatial buffer.
    5. Publish the fused perceived cones
    """

    def __init__(self, name: str):
        super().__init__(name)

        perceived_cones_lidar_topic = "/perceived_cones/lidar"
        perceived_cones_fused_topic = "/perceived_cones/fused"
        perceived_cones_camera_topic = "/perceived_cones/camera"

        self.odom_frame_id = "odom"
        self.common_frame_id = self.odom_frame_id

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, qos=qos_profile_sensor_data
        )
        self.perceived_cones_lidar_filter: TFMessageFilter[idi.ConeMap] = TFMessageFilter(
            tf_listener=self.tf_listener,
            target_frame=self.common_frame_id,
            callback=self.assign_cone_types_and_republish,
            queue_size=10,
        )
        self.perceived_cones_camera_filter: TFMessageFilter[idi.ConeMap] = TFMessageFilter(
            tf_listener=self.tf_listener,
            target_frame=self.common_frame_id,
            callback=self.put_camera_observations_into_buffer,
            queue_size=10,
        )

        self.cone_store = DynamicConeStore()
        self.declare_parameter(
            name="cone_store_eviction_period",
            value=3.0,
            descriptor=ParameterDescriptor(
                name="cone_store_eviction_period",
                type=rclpy.parameter.ParameterType.PARAMETER_DOUBLE, # type: ignore
                description="Time in seconds before a camera reading is forgotten",
            ),
        )

        self.cone_store_time_added: Dict[int, rclpy.time.Time] = {}

        self.perceived_cones_fused_pub = self.create_publisher(  # type: ignore
            idi.ConeMap, perceived_cones_fused_topic, qos_profile=1
        )

        self.perceived_cones_lidar_sub = self.create_subscription(  # type: ignore
            idi.ConeMap,
            perceived_cones_lidar_topic,
            self.perceived_cones_lidar_filter.insert,
            qos_profile=1,
        )

        self.perceived_cones_camera_sub = self.create_subscription(  # type: ignore
            idi.ConeMap,
            perceived_cones_camera_topic,
            self.perceived_cones_camera_filter.insert,
            qos_profile=1,
        )

    def put_camera_observations_into_buffer(self, msg: idi.ConeMap):
        """
        Add the new cones readings from the camera to the spatial cone store,
        and forget previous readings that are too old.
        """
        for cone, cone_type in itertools.chain(
            zip(msg.left_cones, itertools.repeat(ConeType.LEFT)),
            zip(msg.right_cones, itertools.repeat(ConeType.RIGHT)),
            zip(msg.big_orange_cones, itertools.repeat(ConeType.BIG_ORANGE)),
            zip(msg.small_orange_cones, itertools.repeat(ConeType.SMALL_ORANGE)),
        ):
            cone_id = self.cone_store.add_cone(
                cone.position.x, cone.position.y, cone_type
            )
            self.cone_store_time_added[cone_id] = self.get_clock().now()

        self.forget_old_camera_readings()

    def forget_old_camera_readings(self):
        """
        Remove from the spatial cone store all cones that were added there
        more than `cone_store_eviction_period` seconds ago.
        """
        eviction_period: int = int(self.get_parameter("cone_store_eviction_period").get_parameter_value().double_value)  # type: ignore

        removed_ids: List[int] = []
        for cone_id, inserted_time in self.cone_store_time_added.items():
            if self.get_clock().now() - inserted_time > rclpy.duration.Duration(
                seconds=eviction_period
            ):
                self.cone_store.remove_cone(cone_id)
                removed_ids.append(cone_id)

        for cone_id in removed_ids:
            del self.cone_store_time_added[cone_id]

    def assign_cone_types_and_republish(self, msg: idi.ConeMap):
        if not self.cone_store.cones:
            self.get_logger().info("Can't assign types to cones: no readings from camera")  # type: ignore
            return

        def assign_type(c: idi.Cone) -> Tuple[ConeType, idi.Cone]:
            cone_id = self.cone_store.get_nearest_id(c.position.x, c.position.y)
            assert cone_id is not None

            _, _, cone_type = self.cone_store.cones[cone_id]
            return cone_type, c

        all_cones_iter: itertools.chain[idi.Cone] = itertools.chain(msg.left_cones, msg.right_cones, msg.big_orange_cones, msg.small_orange_cones)  # type: ignore
        cones_by_type = {
            t: [c for _, c in cones]
            for t, cones in itertools.groupby(
                sorted(map(assign_type, all_cones_iter), key=lambda e: e[0].value),
                key=lambda e: e[0],
            )
        }

        m = idi.ConeMap(
            header=msg.header,  # type: ignore
            left_cones=cones_by_type.get(ConeType.LEFT, []),
            right_cones=cones_by_type.get(ConeType.RIGHT, []),
            big_orange_cones=cones_by_type.get(ConeType.BIG_ORANGE, []),
            small_orange_cones=cones_by_type.get(ConeType.SMALL_ORANGE, []),
        )

        self.perceived_cones_fused_pub.publish(m)


class PPoint(Protocol):
    x: float
    y: float


class PCone(Protocol):
    position: PPoint


class ConeType(Enum):
    LEFT = "left"
    RIGHT = "right"
    BIG_ORANGE = "big_orange"
    SMALL_ORANGE = "small_orange"


class DynamicConeStore:
    """A helper class supporting adding cones, updating them and finding
    the nearest one to a given position"""

    def __init__(self):
        self._rtree = rtree.index.Index()
        self.next_free_id = 0
        self.cones: Dict[int, Tuple[float, float, ConeType]] = {}

    def add_cone(self, x: float, y: float, cone_type: ConeType) -> int:
        inserted_id = self.next_free_id
        self.next_free_id += 1
        self._rtree.insert(inserted_id, (x, y, x, y))  # type: ignore
        self.cones[inserted_id] = (x, y, cone_type)
        return inserted_id

    def remove_cone(self, id: int):
        x, y, _ = self.cones[id]
        self._rtree.delete(id, (x, y, x, y))  # type: ignore
        del self.cones[id]

    def update_cone(self, id: int, new_x: float, new_y: float) -> None:
        x, y, cone_type = self.cones[id]
        self._rtree.delete(id, (x, y, x, y))  # type: ignore
        self._rtree.insert(id, (new_x, new_y, new_x, new_y))  # type: ignore
        self.cones[id] = new_x, new_y, cone_type

    def get_nearest_id(self, x: float, y: float) -> Optional[int]:
        return next(
            self._rtree.nearest((x, y, x, y), num_results=1), None  # type: ignore
        )

    def clear(self) -> None:
        self._rtree = rtree.index.Index()
        self.cones = {}


def main(args: List[str] = sys.argv):
    rclpy.init(args=args)
    try:
        rclpy.spin(FusionColorLookup("sensor_fusion"))
    except KeyboardInterrupt:
        pass
    except RuntimeError as e: 
        if not "make_tuple(): unable to convert arguments to Python object" in str(e):
            raise e
    finally:
        rclpy.shutdown()
