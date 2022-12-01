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
import rclpy.utilities
import rclpy.node
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import numpy as np

import pygame
from pygame.locals import K_UP, K_DOWN, K_LEFT, K_RIGHT, K_w, K_s, K_a, K_d

class InputCommand:
    def __init__(self, steer: float, speed: float):
        self.steer = steer
        self.speed = speed

class AckermannDriveKeyop(rclpy.node.Node):

    def __init__(self, name: str, max_speed: float, max_steering_angle: float, cmd_topic: str):
        super().__init__(name)

        self.max_speed = max_speed
        self.max_steering_angle = max_steering_angle
        self.max_adjusted_steering_angle = max_steering_angle

        self.ACCELERATION = 0.3
        self.DECELERATION = 0.5


        self.speed: float = 0.0
        self.steering_angle: float = 0.0
        self.drive_pub = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic=cmd_topic,
            qos_profile=1
        )

        self.command = InputCommand(steer=0, speed=0)
        self.last_steering_update = self.get_clock().now()
        self.loop_period = 0.033
        self.create_timer(self.loop_period, self.loop)

    def publish_drive_message(self):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = float(self.speed)
        ackermann_cmd_msg.drive.steering_angle = float(self.steering_angle)
        self.drive_pub.publish(ackermann_cmd_msg)

    def max_steering_angle_at_speed(self, speed):
        return self.max_steering_angle * (0.3 + 0.7 * (1 - np.abs(speed) / self.max_speed))

    def loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit() 
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key==K_UP or event.key==K_w:
                    self.command.speed = 1
                elif event.key==K_LEFT or event.key==K_a:
                    self.command.steer = 1
                elif event.key==K_DOWN or event.key==K_s:
                    self.command.speed = -1
                elif event.key==K_RIGHT or event.key==K_d:
                    self.command.steer = -1
            if event.type == pygame.KEYUP:
                if event.key==K_UP or event.key==K_w:
                    self.command.speed = 0
                elif event.key==K_LEFT or event.key==K_a:
                    self.command.steer = 0
                elif event.key==K_DOWN or event.key==K_s:
                    self.command.speed = 0
                elif event.key==K_RIGHT or event.key==K_d:
                    self.command.steer = 0

        now = self.get_clock().now()
        delta = (now - self.last_steering_update).nanoseconds / 1e9

        if abs(self.command.steer) < 0.01:
            if abs(self.steering_angle) > 0.1:
                self.steering_angle -= delta * 3 * np.sign(self.steering_angle) * self.max_steering_angle
            else:
                self.steering_angle = 0
        else:
            self.steering_angle += delta * 2 * self.command.steer
        
        if abs(self.command.speed) < 0.01:
            if abs(self.speed) > 0.1:
                self.speed -= delta * self.DECELERATION * np.sign(self.speed) * self.max_speed
            else:
                self.speed = 0
        else:
            self.speed += delta * self.ACCELERATION * self.command.speed * self.max_speed

        self.speed = np.clip(
            self.speed, 
            -self.max_speed, 
            self.max_speed)

        self.max_adjusted_steering_angle = self.max_steering_angle_at_speed(self.speed)

        self.steering_angle = np.clip(
            self.steering_angle,
            -self.max_adjusted_steering_angle,
            self.max_adjusted_steering_angle)

        self.last_steering_update = now

        self.publish_drive_message()

    def finalize(self):
        self.get_logger().info('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0.0
        ackermann_cmd_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(ackermann_cmd_msg)

        sys.exit()

def main():
    pygame.init()
    pygame.display.set_mode((100, 100))

    rclpy.init(args=sys.argv)

    args = rclpy.utilities.remove_ros_args(sys.argv[1:])

    max_speed = 5.0
    max_steering_angle = 0.34
    cmd_topic = 'ackermann_drive_commands'

    if len(args) >= 1:
        max_speed = float(args[0])
        max_steering_angle = float(args[0])
    
    if len(args) >= 2:
        max_steering_angle = float(args[1])  

    if len(args) >= 3:
        cmd_topic = f'/{args[2]}'
        
    keyop_node = AckermannDriveKeyop('ackermann_drive_keyop', max_speed, max_steering_angle, cmd_topic)
    
    rclpy.spin(keyop_node)
    
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
