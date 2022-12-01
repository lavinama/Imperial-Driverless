#!/usr/bin/env python3

'''
ackermann_drive_keyop.py:
    A ROS 2 joystick teleoperation script for ackermann steering based robots
'''

__author__ = 'Mateusz Lichota'
__license__ = 'GPLv3'
__maintainer__ = 'Mateusz Lichota'
__email__ = 'mateusz@lichota.net'

import rclpy
import rclpy.node
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
import sys

class AckermannDriveJoyop(rclpy.Node):

    def __init__(self,  max_speed: float, max_steering_angle: float, cmd_topic: str):
        super().__init__('ackermann_drive_joyop')
        
        self.max_speed = max_speed
        self.max_steering_angle = max_steering_angle

        self.speed = 0
        self.steering_angle = 0
        self.drive_pub = self.create_publisher(
            msg_type=AckermannDrive,
            topic=cmd_topic,
            qos_profile=1
        )
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 1)

        self.create_timer(0.2, self.pub_callback)
        self.print_state()

    def joy_callback(self, joy_msg):
        self.speed = joy_msg.axes[2] * self.max_speed
        self.steering_angle = joy_msg.axes[3] * self.max_steering_angle


    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = self.speed
        ackermann_cmd_msg.steering_angle = self.steering_angle
        self.drive_pub.publish(ackermann_cmd_msg)
        self.print_state()

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        self.get_logger().info('\x1b[1M\r'
                      f'\033[34;1mSpeed: \033[32;{self.speed:.2f} m/s, '
                      f'\033[34;1mSteer Angle: \033[32;{self.steering_angle:.2f} rad\033[0m')

    def finalize(self):
        self.get_logger().info('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = 0
        ackermann_cmd_msg.steering_angle = 0
        self.drive_pub.publish(ackermann_cmd_msg)
        sys.exit()

def main():
    rclpy.init(args=sys.argv)

    args = sys.argv[1:]

    max_speed = 0.2
    max_steering_angle = 0.7
    cmd_topic = 'ackermann_cmd'

    if len(args) >= 1:
        max_speed = float(args[0])
        max_steering_angle = float(args[0])
    
    if len(args) >= 2:
        max_steering_angle = float(args[1])  

    if len(args) >= 3:
        cmd_topic = f'/{args[2]}'
        
    joyop_node = AckermannDriveJoyop(max_speed, max_steering_angle, cmd_topic)

    rclpy.spin(joyop_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
