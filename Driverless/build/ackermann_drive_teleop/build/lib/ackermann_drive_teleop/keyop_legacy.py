#!/usr/bin/env python3

'''
ackermann_drive_keyop.py:
    A ROS 2 keyboard teleoperation script for ackermann steering based robots
'''

__author__ = 'Mateusz Lichota'
__license__ = 'GPLv3'
__maintainer__ = 'Mateusz Lichota'
__email__ = 'mateusz@lichota.net'

import rclpy
import rclpy.node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import sys, select, termios, tty
import curses
import numpy as np

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09',
    'ctrl+c': '\x03',
    'q'     : '\x71'}

key_bindings = {
    '\x41' : ( 1.0 , 0.0),
    '\x42' : (-1.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0),
    '\x44' : ( 0.0 , 1.0),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}


class AckermannDriveKeyop(rclpy.node.Node):

    def __init__(self, max_speed: float, max_steering_angle: float, cmd_topic: str):
        super().__init__('ackermann_drive_keyop')

        self.speed_range = [-max_speed, max_speed]
        self.steering_angle_range = [-max_steering_angle, max_steering_angle]
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * max_speed / 5,
                     key_bindings[key][1] * max_steering_angle / 5)

        self.speed: float = 0.0
        self.steering_angle: float = 0.0
        self.drive_pub = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic=cmd_topic,
            qos_profile=1
        )

        self.screen = curses.initscr()
        curses.echo(False)
        self.screen.nodelay(True)

        self.create_timer(0.2, self.publish_drive_message)

        self.last_steering_update = self.get_clock().now()
        self.keyboard_loop_period = 0.01
        self.create_timer(self.keyboard_loop_period, self.loop)
        self.print_state()

    def publish_drive_message(self):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        # sys.stdout.write('\033[2J')
        self.get_logger().info('\x1b[1M\r*********************************************')
        self.get_logger().info('\x1b[1M\rUse arrows to change speed and steering angle')
        self.get_logger().info('\x1b[1M\rUse space to brake and tab to align wheels')
        self.get_logger().info('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        self.get_logger().info('\x1b[1M\r*********************************************')
        self.get_logger().info('\x1b[1M\r'
                      f'\033[34;1mSpeed: \033[32;1m{self.speed:.2f} m/s, '
                      f'\033[34;1mSteer Angle: \033[32;1m{self.steering_angle:.2f} rad\033[0m')

    def loop(self):

        now = self.get_clock().now()
        delta = (now - self.last_steering_update).nanoseconds / 1e9
        self.steering_angle = self.steering_angle - delta * 1 * np.sign(self.steering_angle) * self.steering_angle_range[1]
        self.speed = self.speed - delta * 0.2 * np.sign(self.speed) * self.speed_range[1]
        self.last_steering_update = now

        # self.print_state()

        try:
            key = self.screen.getkey()
        
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.steering_angle = self.steering_angle + key_bindings[key][1]
                    self.speed = np.clip(
                        self.speed, self.speed_range[0], self.speed_range[1])
                    self.steering_angle = np.clip(
                        self.steering_angle,
                        self.steering_angle_range[0],
                        self.steering_angle_range[1])

                    self.last_steering_update = self.get_clock().now()

                    self.publish_drive_message()
        except KeyboardInterrupt:
            self.finalize()
        except: 
            pass

    def finalize(self):
        self.get_logger().info('Halting motors, aligning wheels and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0.0
        ackermann_cmd_msg.drive.steering_angle = 0.0
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
        
    keyop_node = AckermannDriveKeyop(max_speed, max_steering_angle, cmd_topic)
    
    
    try:
        rclpy.spin(keyop_node)
    except:
        pass
    finally:
        curses.nocbreak()
        curses.echo(True)
        curses.endwin()
    
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
