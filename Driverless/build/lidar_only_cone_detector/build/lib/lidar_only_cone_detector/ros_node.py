from typing import Type
from itertools import starmap

import rclpy
import rclpy.node
import rclpy.qos

import sensor_msgs.msg
import imperial_driverless_interfaces.msg as idi
import geometry_msgs.msg

from .cone_detector_base import ConeDetectorBase


class ConeDetectorROSNode(rclpy.node.Node):
    def __init__(self, name: str, cone_detector_implementation: Type[ConeDetectorBase]):
        super().__init__(name)  # type: ignore

        scan_topic_name = 'scan'
        perceived_cones_topic_name = '/perceived_cones/lidar'

        self.subscription = self.create_subscription(  # type: ignore
            sensor_msgs.msg.LaserScan,
            scan_topic_name,
            self.new_scan_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )


        self.perceived_cones_pub = self.create_publisher( # type: ignore
            idi.ConeMap, 
            perceived_cones_topic_name,
            qos_profile=10)

        self.cone_detector = None
        self.cone_detector_implementation: Type[
            ConeDetectorBase
        ] = cone_detector_implementation

    def new_scan_callback(self, scan_msg: sensor_msgs.msg.LaserScan):
        if self.cone_detector is None:
            self.cone_detector = self.cone_detector_implementation(
                scan_msg.angle_min,
                scan_msg.angle_max,
                scan_msg.range_min,
                scan_msg.range_max,
                scan_msg.angle_increment,
            )

        left_cones, right_cones = self.cone_detector.detect_cones(list(scan_msg.ranges))

        msg = idi.ConeMap()
        msg.header = scan_msg.header  # type: ignore
        msg.left_cones = list(starmap(make_idi_cone, left_cones))
        msg.right_cones = list(starmap(make_idi_cone, right_cones))
        self.perceived_cones_pub.publish(msg)

def make_idi_cone(x: float, y: float) -> idi.Cone:
    return idi.Cone(position=geometry_msgs.msg.Point(x=float(x), y=float(y), z=0.0))


def expose_as_ros_node(cone_detector_implementation: Type[ConeDetectorBase], name: str):
    rclpy.init(args=None)
    try:
        rclpy.spin(ConeDetectorROSNode(name, cone_detector_implementation))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
