import sys
from typing import Optional, Type
import rclpy
import rclpy.time
import rclpy.node
import rclpy.qos
import geometry_msgs.msg
import nav_msgs.msg
import imperial_driverless_interfaces.msg
import math

from tf2_ros import Buffer, TransformListener


from .vehicle_controller_base import VehicleControllerBase

class VehicleControllerROSNode(rclpy.node.Node):

    def __init__(self, name: str, vehicle_controller_class: Type[VehicleControllerBase]):
        super().__init__(name) # type: ignore

        self.max_steer_angle_deg = 21
        self.max_throttle_nm = 195
        self.max_brake_pct = 100

        cmd_vel_topic = 'cmd_vel'
        vcu_drive_cmd_topic = 'vcu_drive_cmd'
        odom_topic = 'odom'

        self._cmd_vel_sub = self.create_subscription( # type: ignore
            geometry_msgs.msg.Twist,
            cmd_vel_topic,
            self.new_vel_command_callback, 
            qos_profile=1
        )

        self._odom_sub = self.create_subscription( # type: ignore
            nav_msgs.msg.Odometry,
            odom_topic,
            self.odometry_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        self._vcu_drive_cmd_pub = self.create_publisher( # type: ignore
            imperial_driverless_interfaces.msg.VCUDriveCommand, 
            vcu_drive_cmd_topic,
            qos_profile=1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._vehicle_controller_class = vehicle_controller_class
        self._vehicle_controller: Optional[VehicleControllerBase] = None
        self._vehicle_controller_init_timer = self.create_timer(1, self._try_initialize_vehicle_controller)

    def _try_initialize_vehicle_controller(self):
        try:
            ts = self.tf_buffer.lookup_transform(
                'base_link', 
                'left_steering_hinge', 
                rclpy.time.Time(), 
            )
            wheelbase = ts.transform.translation.x
            assert isinstance(wheelbase, float)
            self._vehicle_controller = self._vehicle_controller_class(wheelbase)
            self._vehicle_controller_init_timer.destroy()

        except Exception as e:
            if 'frame does not exist.' in str(e):
                return None
            else:
                raise e
        

    def new_vel_command_callback(self, msg: geometry_msgs.msg.Twist):
        if self._vehicle_controller is None:
            return

        self._vehicle_controller.set_velocity_targets(msg.linear.x, msg.angular.z)

    def _clamp(self, val: float, min_val: int, max_val: int) -> int:
        return round(max(min(val, max_val), min_val))

    def odometry_callback(self, msg: nav_msgs.msg.Odometry):
        if self._vehicle_controller is None:
            return

        velocity_norm = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self._vehicle_controller.incorporate_velocity_feedback(velocity_norm)

        steer_rad, throttle_f, throttle_r, brake_f, brake_r = self._vehicle_controller.get_control_commands()
        steer_deg = self._clamp(math.degrees(steer_rad), -self.max_steer_angle_deg, self.max_steer_angle_deg)
    
        vcu_drive_cmd = imperial_driverless_interfaces.msg.VCUDriveCommand()
        vcu_drive_cmd.steer_request_deg = steer_deg
        vcu_drive_cmd.front_axle_trq_request = self._clamp(throttle_f, 0, self.max_throttle_nm)
        vcu_drive_cmd.rear_axle_trq_request = self._clamp(throttle_r, 0, self.max_throttle_nm)
        vcu_drive_cmd.hyd_press_f_req_pct = self._clamp(brake_f, 0, self.max_brake_pct)
        vcu_drive_cmd.hyd_press_r_req_pct = self._clamp(brake_r, 0, self.max_brake_pct)
        self._vcu_drive_cmd_pub.publish(vcu_drive_cmd)
    

def expose_as_ros_node(vehicle_controller_class: Type[VehicleControllerBase], name: str):
    rclpy.init(args=sys.argv[1:])
    try:
        rclpy.spin(VehicleControllerROSNode(name, vehicle_controller_class))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
