import itertools
from typing import Iterator, List, Tuple, TypeVar, Optional

from .cone_detector_base import ConeDetectorBase, Cone

from .ros_node import expose_as_ros_node

from statistics import median
from math import sin, cos, atan
import numpy as np # type: ignore

MAX_R_SEP = 1.0
CONE_RADIUS = 0.15
MINIMUM_CONES_PER_GROUP = 3
MAX_VARIANCE = 0.005

from typing import List, Tuple, Optional
T = TypeVar("T")

class CircleApproximationDetector(ConeDetectorBase):
    def __init__(self, angle_min: float, angle_max: float, range_min: float, range_max: float, angle_increment: float):
        super().__init__(angle_min, angle_max, range_min, range_max, angle_increment)

    def detect_cones(self, ranges: List[float]) -> Tuple[List[Cone], List[Cone]]:
        group_rs = [ranges[0]]
        group_ths = [self.angle_min]

        def is_group_a_cone(group_rs: List[float]) -> bool:
            max_points_on_cone = (
                2 + 2 * atan(CONE_RADIUS / max(group_rs)) / self.angle_increment
            )
            max_spread = 2 * CONE_RADIUS

            too_many_points = len(group_rs) > max_points_on_cone
            too_big_spread = max(group_rs) - min(group_rs) > max_spread

            return (
                not too_many_points
                and not too_big_spread
                and len(group_rs) >= MINIMUM_CONES_PER_GROUP
            )

        cones: List[Cone] = []

        def skip_first(it: Iterator[T]) -> Iterator[T]:
            return itertools.islice(it, 1, None)

        def polar_to_xy(r: float, th: float):
            return (r * cos(th), r * sin(th))

        def circle_from_points(rs: List[float], ths: List[float]):
            ps = [polar_to_xy(r, th) for r, th in zip(rs, ths)]
            return find_circle(ps)

        def find_circle(ps: List[Cone]) -> Optional[Cone]:
            point_combinations = itertools.combinations(ps, 2)
            x_coords: List[float] = []
            y_coords: List[float] = []
            for (point1, point2) in point_combinations:
                x, y = circle_center(point1, point2)
                x_coords.append(x)
                y_coords.append(y)
            x_variance: float = np.var(x_coords) # type: ignore
            y_variance: float = np.var(y_coords) # type: ignore

            if not (x_variance > MAX_VARIANCE or y_variance > MAX_VARIANCE):
                return (sum(x_coords) / len(x_coords), sum(y_coords) / len(y_coords))
            else:
                return None

        def circle_center(p1: Cone, p2: Cone) -> Cone:
            midpoint = middle_point(p1, p2)

            half_chord_length = magnitude((p2[0] - p1[0], p2[1] - p1[1])) / 2

            # even though cone radius is 0.15, algorithm overshoots and works best at r=0.1
            missing_radius = calculate_missing_right_angle_triangle_side(0.1, half_chord_length)

            clockwise_rotation_vector = vector_of_magnitude((p2[1] - p1[1], p1[0] - p2[0]), missing_radius)
            anticlockwise_rotation_vector = vector_of_magnitude((p1[1] - p2[1], p2[0] - p1[0]), missing_radius)

            anticlockwise_candidate = add_vector_to_point(anticlockwise_rotation_vector, midpoint)
            clockwise_candidate = add_vector_to_point(clockwise_rotation_vector, midpoint)

            # from two points there are two possible circles: one closer and one further away from the robot
            # the candidate that is further away from the robot is the one we are interested in
            if magnitude(anticlockwise_candidate) > magnitude(clockwise_candidate):
                return anticlockwise_candidate
            else:
                return clockwise_candidate

        def calculate_missing_right_angle_triangle_side(hyp: float, side: float) -> float:
            if side > hyp:
                return 0
            else:
                return (hyp**2 - side**2) ** 0.5

        def magnitude(p: Tuple[float, float]) -> float:
            return (p[0] ** 2 + p[1] ** 2) ** 0.5

        def vector_of_magnitude(vector: Tuple[float, float], d: float) -> Tuple[float, float]:
            vector_scalar = d / magnitude(vector)
            return (vector[0] * vector_scalar, vector[1] * vector_scalar)

        def middle_point(p1: Cone, p2: Cone) -> Cone:
            return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

        def add_vector_to_point(vector: Tuple[float, float], point: Cone):
            return (point[0] + vector[0], point[1] + vector[1])

        thetas = itertools.count(self.angle_min, self.angle_increment)
        for r, th in skip_first(zip(ranges, thetas)):
            if abs(r - median(group_rs)) > MAX_R_SEP:
                if is_group_a_cone(group_rs):
                    if (x := circle_from_points(group_rs, group_ths)) is not None:
                        cones.append(x)
                group_rs, group_ths = [r], [th]
            else:
                group_rs.append(r)
                group_ths.append(th)
        return cones, []

def main():
    expose_as_ros_node(CircleApproximationDetector, "circle_approximation_detector")
