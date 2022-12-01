import math
from typing import Tuple

from simple_pid.PID import PID

from .vehicle_controller_base import (VehicleControllerBase, MetresPerSecond, 
    RadiansPerSecond, NewtonMetres, BreakStrength, Metres, Radians)

from .ros_node import expose_as_ros_node

class BasicPID(VehicleControllerBase):
    def __init__(self, wheelbase: Metres):
        self.most_recent_velocity: MetresPerSecond = 0.0

        self.max_braking_torque: NewtonMetres = -100.0

        self.pid = PID(5, 0.1, 1, setpoint=0, output_limits=(-300, 150))
        self.wheelbase = wheelbase

        self.linear_velocity_target = 0.0
        self.angular_velocity_target = 0.0

    def incorporate_velocity_feedback(self, velocity: MetresPerSecond):
        self.most_recent_velocity = velocity

    def set_velocity_targets(self, linear: MetresPerSecond, angular: RadiansPerSecond):
        self.pid.setpoint = linear
        self.linear_velocity_target = linear
        self.angular_velocity_target = angular

    def _compute_steering_angle(self):
        if self.most_recent_velocity != 0.0:
            return math.atan(self.wheelbase * self.angular_velocity_target / self.most_recent_velocity)
        else:
            return 0.0

    def _compute_brake_pct_for_given_torque(self, torque: NewtonMetres) -> BreakStrength:
        assert torque <= 0.0, "If braking, torque must be negative"
        return 100 * torque / self.max_braking_torque

    def _compute_throttle_and_brakes(self) -> Tuple[NewtonMetres, BreakStrength]:
        desired_torque = self.pid(self.most_recent_velocity)
        assert isinstance(desired_torque, float)

        if desired_torque >= 0:
            return (desired_torque, 0.0)
        else:
            return (0.0, self._compute_brake_pct_for_given_torque(desired_torque))



    def get_control_commands(self) -> Tuple[Radians, NewtonMetres, NewtonMetres, BreakStrength, BreakStrength]:
        steering_angle = self._compute_steering_angle()
        throttle, brakes = self._compute_throttle_and_brakes()
        return steering_angle, throttle, throttle, brakes, brakes


def main():
    expose_as_ros_node(BasicPID, 'basic_pid_vehicle_controller')
