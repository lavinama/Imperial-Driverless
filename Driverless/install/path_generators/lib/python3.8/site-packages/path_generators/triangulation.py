from typing import Dict, Iterable, Sequence, List, Set, Tuple

import numpy as np
import numpy.linalg
import scipy.spatial

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from .path_generator_base import PathGeneratorBase, PCone, Point2D
from .ros_node import expose_as_ros_node

from collections import deque
from itertools import chain

Metres = float

class TraingPath(PathGeneratorBase):
    def __init__(self):
        super().__init__()
        self._max_distance_between_midpoints: Metres = 5.0
        self.left_track_edges = []
        self.right_track_edges = []
        self.other_edges: Set[Tuple[Point2D, Point2D]] = set()
    
    def generate_path(self, left_cones: Sequence[PCone], right_cones: Sequence[PCone]) -> Sequence[Point2D]:
        if len(left_cones) + len(right_cones) < 4: # scipy.spatial.Delaunay requires at least 4 points. See issue #7 on GitHub.
            return []
        
        left_cones_set = {(p.x, p.y) for p in left_cones}
        
        all_cones_np = np.array([(p.x, p.y) for p in chain(left_cones, right_cones)])
        
        triangulation = scipy.spatial.Delaunay(all_cones_np)
        triangles = all_cones_np[triangulation.simplices][::-1] # type: ignore

        self.other_edges = set()
        self.left_track_edges = []
        self.right_track_edges = []
        midpoint_pairs: List[Tuple[Point2D, Point2D]] = []
        for t in triangles:
            assert len(t) == 3 # Each triangle has three vertices
            assert all(map(lambda x: x.shape == (2,), t)) # Each vertex is a point with two coordinates

            vertex_types = [tuple(vertex) in left_cones_set for vertex in t] # True - left, False - right
            num_left = vertex_types.count(True)
            
            # Ignore triangles made of three cones of the same colour
            if num_left == 0 or num_left == 3:
                continue 

            # Assuming not all cones in a triangle are of the same colour,
            # there's one 'single cone', and two 'other cones'.
            single_vertex_is_left = (num_left == 1)
            single_cone = vertex_types.index(True if single_vertex_is_left else False)
            other_cones: Tuple[int, int] = [(1, 2), (0, 2), (0, 1)][single_cone]

            # Naming convention: S is the single cone, A and B are the other two.
            # A is supposed to be before B in the track order, but is not guaranteed
            # to be at this stage.
            A_idx, B_idx = other_cones
            A = (t[A_idx][0], t[A_idx][1])
            B = (t[B_idx][0], t[B_idx][1])
            S = (t[single_cone][0], t[single_cone][1])

            # Equation from https://stackoverflow.com/a/1560510/16975344
            single_on_the_left_of_AB = 0 <= ((B[0] - A[0]) * (S[1] - A[1]) - (B[1] - A[1]) * (S[0] - A[0]))

            # Logical xor here ensures that we only switch when
            # - single vertex is a left cone, and it is on the right of vector A->B, or
            # - single vertex is a right cone, and it is on the left of vector A->B
            should_switch_AB = single_on_the_left_of_AB ^ single_vertex_is_left

            if should_switch_AB:
                A, B = B, A

            AS_midpoint = (A[0] + S[0]) / 2, (A[1] + S[1]) / 2
            BS_midpoint = (B[0] + S[0]) / 2, (B[1] + S[1]) / 2

            distance_between_midpoints = numpy.linalg.norm(np.array(AS_midpoint) - np.array(BS_midpoint))

            if distance_between_midpoints > self._max_distance_between_midpoints:
                continue
            
            midpoint_pairs.append((AS_midpoint, BS_midpoint))
                
            track_side_edges = self.right_track_edges if single_vertex_is_left else self.left_track_edges
            track_side_edges.append((A, B))

            # Track side edges won't contain duplicates (not sure if that's guaranteed,
            # but it's highly unlikely). However, every 'other' edge (between cones of
            # different colours) will occur twice. Hence, we need to pay attention to
            # not include duplicates. We do that by storing them as tuples in a set.
            # Those tuples could have different orders of elements, so we ensure that
            # the order of the two edge endpoints is always the same by sorting them.
            self.other_edges.add((A, S) if A <= S else (S, A))
            self.other_edges.add((B, S) if B <= S else (S, B))
            
            
        
        # Find path through midpoints
        return self._connect_midpoint_pairs(midpoint_pairs)

    def _connect_midpoint_pairs(self, midpoint_pairs: List[Tuple[Point2D, Point2D]]) -> List[Point2D]:
        if not midpoint_pairs:
            return []
        
        next_mids: Dict[Point2D, Point2D] = {}
        prev_mids: Dict[Point2D, Point2D] = {}
        
        for pt1, pt2 in midpoint_pairs:
            next_mids[pt1] = pt2
            prev_mids[pt2] = pt1

        arbitrary_initial_midpoint = next(iter(next_mids))

        # it's guaranteed that the arbitrary initial midpoint has a next midpoint
        curr_mid = next_mids[arbitrary_initial_midpoint]
        
        path = deque([arbitrary_initial_midpoint])

        # Expand forwards
        path.append(curr_mid)
        while curr_mid in next_mids:
            curr_mid = next_mids[curr_mid]
            path.append(curr_mid)
            if curr_mid == arbitrary_initial_midpoint:
                break
            
        else:
            # This block runs only if we didn't break from the while loop.
            # Hence, the path does not contain loop and we should expand backwards
            if arbitrary_initial_midpoint in prev_mids:
                curr_mid = prev_mids[arbitrary_initial_midpoint]

                path.appendleft(curr_mid)
                while curr_mid in prev_mids:
                    curr_mid = prev_mids[curr_mid]
                    path.appendleft(curr_mid)

        return list(path)

    def get_visualization(self) -> List[Marker]:
        def lsm(point_pairs: Iterable[Tuple[Point2D, Point2D]], color, thickness=1.0):

            points = chain(*point_pairs)

            lsm.i += 1
            m = Marker()
            m.header.frame_id = "map"
            m.type = Marker.LINE_LIST
            m.id=lsm.i
            m.action = Marker.ADD
            m.scale.x = 0.05 * thickness
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 1.0
            m.points = [Point(x=p[0], y=p[1], z=0.001) for p in points]
            return m

        lsm.i = 0

        return [
            lsm(self.left_track_edges, (0.0, 0.0, 1.0), 2),
            lsm(self.right_track_edges, (1.0, 1.0, 0.0), 2),
            lsm(self.other_edges, (0.5, 0.5, 0.5), 1),
        ]

def main():
    expose_as_ros_node(TraingPath(), 'path_generator_triangulation')




if __name__ == '__main__':
    import dataclasses
    import id_track_utils
    from id_track_utils import Track
    from matplotlib import pyplot as plt

    @dataclasses.dataclass
    class Cone:
        x: float
        y: float
        x_std_dev: float = 0.0
        y_std_dev: float = 0.0

    ex_track: Track = id_track_utils.example_tracks['small_track']

    # ex_track.as_occupancy_grid()[0].show()

    left_cones=[Cone(x, y) for x, y in ex_track.left_cones]
    right_cones=[Cone(x, y) for x, y in ex_track.right_cones]

    t = TraingPath()

    path = t.generate_path(left_cones,  right_cones)
    xs, ys = tuple(zip(*path))
    plt.plot(xs, ys, 'green')


    for cone_list, colour in (
        (t.left_track_edges, 'b'),
        (t.right_track_edges, 'y'),
        (t.other_edges, 'gray')
    ):
        for cone in cone_list:
            plt.plot([cone[0][0], cone[1][0]], [cone[0][1], cone[1][1]], colour)
        
    plt.show()
