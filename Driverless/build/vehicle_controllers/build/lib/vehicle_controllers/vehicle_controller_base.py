from abc import abstractmethod, ABC
from typing import Tuple

MetresPerSecond = float
RadiansPerSecond = float
Metres = float
NewtonMetres = float
BreakStrength = float
Radians = float

class VehicleControllerBase(ABC):
    @abstractmethod 
    def __init__(self, wheelbase: Metres): ...

    @abstractmethod
    def incorporate_velocity_feedback(self, velocity: MetresPerSecond): ...
    
    @abstractmethod
    def set_velocity_targets(self, linear: MetresPerSecond, angular: RadiansPerSecond): ...

    @abstractmethod                      # steering, front motor,  rear motor,   front break,   rear break
    def get_control_commands(self) -> Tuple[Radians, NewtonMetres, NewtonMetres, BreakStrength, BreakStrength]: ...
                                          
