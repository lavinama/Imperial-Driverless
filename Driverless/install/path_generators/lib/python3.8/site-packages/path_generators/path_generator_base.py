from abc import abstractmethod, ABC
from typing import List, Tuple, Protocol, Sequence
from visualization_msgs.msg import Marker

class PCone(Protocol):
    x: float
    y: float
    x_std_dev: float
    y_std_dev: float


Point2D = Tuple[float, float]

class PathGeneratorBase(ABC):
    @abstractmethod
    def generate_path(self, left_cones: Sequence[PCone], right_cones: Sequence[PCone]) -> Sequence[Point2D]: ...

    @abstractmethod
    def get_visualization(self) -> List[Marker]:
        pass
