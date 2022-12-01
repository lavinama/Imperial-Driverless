from abc import abstractmethod, ABC
from typing import List, Tuple


Cone = Tuple[float, float]

class ConeDetectorBase(ABC):

    def __init__(self, angle_min: float, angle_max: float, range_min: float, range_max: float, angle_increment: float):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.range_min = range_min
        self.range_max = range_max
        self.angle_increment = angle_increment

    @abstractmethod
    def detect_cones(self, ranges: List[float]) -> Tuple[List[Cone], List[Cone]]: ...
