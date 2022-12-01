from typing import Sequence
import math

from .path_generator_base import PathGeneratorBase, PCone, Point2D

from .ros_node import expose_as_ros_node

class ExampleGenerator(PathGeneratorBase):
    def generate_path(self, left_cones: Sequence[PCone], right_cones: Sequence[PCone]) -> Sequence[Point2D]:
        
        # return a full ellipse trajectory
        return [(10*math.cos(x/100), 20*math.sin(x/100)) for x in range(0, 628)]

    def get_visualization(self):
        return []

        
def main():
    expose_as_ros_node(ExampleGenerator(), 'example_path_generator')
