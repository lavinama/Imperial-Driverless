from typing import List, Sequence, Tuple
import rclpy
import rclpy.time
import rclpy.node
import tf2_ros
import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

import visualization_msgs.msg

import imperial_driverless_interfaces.msg as idi
from .path_generator_base import PathGeneratorBase, Point2D

import numpy as np


class PathGeneratorROSNode(rclpy.node.Node):

    def __init__(self, name: str, path_generator: PathGeneratorBase):
        super().__init__(name)

        cone_map_topic = 'cone_map' 
        path_topic = 'path'

        # introduce parameters for smoothing
        self.declare_parameter('use_smoothing', True)
        self.declare_parameter('smoothing_factor', 0.8)
        self.declare_parameter('sample_density', 10.0) # 1 sample / space unit

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.subscription = self.create_subscription(
            idi.ConeMap, 
            cone_map_topic, 
            self.new_conemap_callback, 
            qos_profile=10
        )

        self.vis_pub = self.create_publisher(
            visualization_msgs.msg.MarkerArray,
            '/visualization/path_generator',
            qos_profile=10)


        self.path_pub = self.create_publisher(
            nav_msgs.msg.Path, 
            path_topic,
            qos_profile=10)

        self.path_generator = path_generator

    def _get_path_len(self,
        xs: List[Tuple[float, float]],
        ys: List[Tuple[float, float]]
    ) -> float:
        xs, ys = np.asarray(xs), np.asarray(ys) # type: ignore
        diff_x = np.square(xs[1:] - xs[:-1])    # type: ignore
        diff_y = np.square(ys[1:] - ys[:-1])    # type: ignore
        return np.sum(np.sqrt(diff_x+diff_y))   # type: ignore

    def _smooth_path(self, 
        path: Sequence[Point2D], 
        smoothing_factor: float, 
        sample_density: int
    ) -> List[Point2D]:

        if not path:
            return []

        xs, ys = tuple(zip(*path))

        # fit data points to scipy bspline
        from scipy import interpolate
        weights = np.ones(len(xs))
        weights[0], weights[-1] = 10, 10 # ensure that the first and last points stay the same
        tck, *others = interpolate.splprep([xs,ys], w=weights, s=smoothing_factor, k=min(len(xs)-1, 3))
        
        # compute number of samples based on total path length
        total_len = self._get_path_len(xs,ys)
        num_samples = int(total_len*sample_density)
        u = np.linspace(0,1, num=num_samples)
        xs, ys = interpolate.splev(u, tck)
        return [(xi,yi) for xi,yi in zip(xs,ys)]

    def new_conemap_callback(self, conemap_msg: idi.ConeMap) -> None:
        
        left_positions = [c.position for c in conemap_msg.left_cones]
        right_positions = [c.position for c in conemap_msg.right_cones]

        path = self.path_generator.generate_path(left_positions, right_positions)

        use_smoothing = self.get_parameter('use_smoothing').value

        # smooth the path if params are given
        if use_smoothing:
            smoothing = self.get_parameter('smoothing_factor').value
            sample_density = self.get_parameter('sample_density').value
            path = self._smooth_path(path, smoothing, sample_density)

        stamp = conemap_msg.header.stamp
        msg = nav_msgs.msg.Path()
        msg.header = std_msgs.msg.Header(stamp=stamp, frame_id=conemap_msg.header.frame_id)
        msg.poses = [point_to_pose_stamped_msg(point, msg.header) for point in path]
        self.path_pub.publish(msg)


        vis = visualization_msgs.msg.MarkerArray()
        markers = self.path_generator.get_visualization()
        for m in markers:
            m.header.stamp = stamp # type: ignore
        vis.markers = markers
        self.vis_pub.publish(vis)

def point_to_pose_stamped_msg(point: Point2D, header: std_msgs.msg.Header) -> geometry_msgs.msg.PoseStamped:
    return geometry_msgs.msg.PoseStamped(
        header=header,
        pose=geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=point[0], y=point[1])
        )
    ) 

def expose_as_ros_node(path_generator_instance: PathGeneratorBase, name: str):
    rclpy.init(args=None)
    try:
        rclpy.spin(PathGeneratorROSNode(name, path_generator_instance))
    except KeyboardInterrupt:
        pass
    except RuntimeError as e: 
        if not "make_tuple(): unable to convert arguments to Python object" in str(e):
            raise e
    finally:
        rclpy.shutdown()
