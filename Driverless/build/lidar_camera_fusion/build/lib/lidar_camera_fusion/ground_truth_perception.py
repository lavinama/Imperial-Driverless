import math
import sys
from typing import List

import rclpy
import rclpy.node
import rclpy.qos

import imperial_driverless_interfaces.msg as idi
import tf2_ros
import rclpy.parameter

from rclpy.qos import qos_profile_sensor_data
import rclpy.time
import rclpy.duration

import copy

from imperial_driverless_utils.tf_message_filter import TFMessageFilter


class GroundTruthPerception(rclpy.node.Node):
    """
    This class simulates perfect cone perception, by using the ground truth car position and cone map
    to calculate which cones would be seen by the car.
    """

    def __init__(self, name: str):
        super().__init__(name)

        ground_truth_cone_map_topic = "/ground_truth/cone_map"
        perceived_cones_fused_topic = "/perceived_cones/fused"

        self.lidar_frame_id = "laser"

        self.frequency = 10.0
        self.simulated_cone_detection_range = 15.0
        self.fov_per_side = 2.10 # radians, means we can see cones in the range of -fov_per_side to fov_per_side

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, qos=qos_profile_sensor_data
        )
        self.tf_filter: TFMessageFilter[idi.ConeMap] = TFMessageFilter(
            tf_listener=self.tf_listener,
            target_frame=self.lidar_frame_id,
            callback=self.filter_out_cones_outside_fov_and_republish,
            queue_size=10,
        )

        self.ground_truth_cone_map_sub = self.create_subscription(  # type: ignore
            idi.ConeMap,
            ground_truth_cone_map_topic,
            self.save_ground_truth_cone_map,
            qos_profile=1,
        )

        self.ground_truth_cone_map: idi.ConeMap = idi.ConeMap()

        self.perceived_cones_fused_pub = self.create_publisher(  # type: ignore
            idi.ConeMap, perceived_cones_fused_topic, qos_profile=1
        )

    def restamp(self, msg: idi.ConeMap) -> idi.ConeMap:
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def republish_timer_callback(self):
        msg = self.restamp(copy.deepcopy(self.ground_truth_cone_map))
        self.tf_filter.insert(msg)

    def save_ground_truth_cone_map(self, msg: idi.ConeMap):
        if self.ground_truth_cone_map_sub:
            self.ground_truth_cone_map_sub = None
            self.ground_truth_cone_map = msg
            self.create_timer(1/self.frequency, self.republish_timer_callback)

    def filter_out_cones_outside_fov_and_republish(self, msg: idi.ConeMap):
        """
        msg MUST be in the lidar frame.
        """

        def is_in_fov(cone: idi.Cone) -> bool:
            bearing = math.atan2(cone.position.y, cone.position.x)
            distance = math.sqrt(cone.position.x ** 2 + cone.position.y ** 2)
            not_too_far = distance < self.simulated_cone_detection_range
            within_fov = bearing > -self.fov_per_side and bearing < self.fov_per_side 
            return not_too_far and within_fov

        m = idi.ConeMap(
            header=msg.header,  # type: ignore
            left_cones=[cone for cone in msg.left_cones if is_in_fov(cone)],
            right_cones=[cone for cone in msg.right_cones if is_in_fov(cone)],
            big_orange_cones=[cone for cone in msg.big_orange_cones if is_in_fov(cone)],
            small_orange_cones=[cone for cone in msg.small_orange_cones if is_in_fov(cone)],
        )

        self.perceived_cones_fused_pub.publish(m)

def main(args: List[str] = sys.argv):
    rclpy.init(args=args)
    try:
        rclpy.spin(GroundTruthPerception("ground_truth_perception"))
    except KeyboardInterrupt:
        pass
    except RuntimeError as e: 
        if not "make_tuple(): unable to convert arguments to Python object" in str(e):
            raise e
    finally:
        rclpy.shutdown()
