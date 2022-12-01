#!/usr/bin/env python3

'''
ackermann_drive_keyop.py:
    A ROS 2 keyboard teleoperation script for ackermann steering based robots
'''

__author__ = 'Mateusz Lichota'
__license__ = 'GPLv3'
__maintainer__ = 'Mateusz Lichota'
__email__ = 'mateusz@lichota.net'

from typing import Callable, TypeVar
import rclpy
import rclpy.utilities
import rclpy.node
import sys

import math

import pygame
from pygame.locals import K_UP, K_DOWN, K_LEFT, K_RIGHT, K_w, K_s, K_a, K_d

T = TypeVar('T')

class CarKeyop(rclpy.node.Node):

    def __init__(self, name: str, max_speed: float, max_steer: float, make_publisher: Callable[[rclpy.node.Node], T], publish_callback: Callable[[T, float, float], None]):
        super().__init__(name)

        self.max_speed = max_speed
        self.max_steer = max_steer

        self.speed_acc = 2.0
        self.steer_acc = 2.0

        self.speed = 0.0
        self.steer = 0.0
        
        self.steer_cmd = 0
        self.speed_cmd = 0

        self.drive_pub = make_publisher(self)
        self.publish_drive_message = lambda: publish_callback(self.drive_pub, self.speed, self.steer)

        self.last_steering_update = self.get_clock().now()
        self.loop_period = 0.033
        self.create_timer(self.loop_period, self.loop)

    def loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit() 
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key==K_UP or event.key==K_w:
                    self.speed_cmd += 1
                elif event.key==K_LEFT or event.key==K_a:
                    self.steer_cmd += 1
                elif event.key==K_DOWN or event.key==K_s:
                    self.speed_cmd -= 1
                elif event.key==K_RIGHT or event.key==K_d:
                    self.steer_cmd -= 1
            if event.type == pygame.KEYUP:
                if event.key==K_UP or event.key==K_w:
                    self.speed_cmd -= 1
                elif event.key==K_LEFT or event.key==K_a:
                    self.steer_cmd -= 1
                elif event.key==K_DOWN or event.key==K_s:
                    self.speed_cmd += 1
                elif event.key==K_RIGHT or event.key==K_d:
                    self.steer_cmd += 1

        if self.speed_cmd == 0:
            self.speed -= math.copysign(min(self.speed_acc * self.loop_period, abs(self.speed)), self.speed)
        else:
            self.speed += self.speed_cmd * self.speed_acc * self.loop_period
            self.speed = max(-self.max_speed, min(self.max_speed, self.speed))

        if self.steer_cmd == 0:
            self.steer -= math.copysign(min(self.steer_acc * self.loop_period, abs(self.steer)), self.steer)
        else:
            self.steer += self.steer_cmd * self.steer_acc * self.loop_period
            self.steer = max(-self.max_steer, min(self.max_steer, self.steer))

        self.publish_drive_message()

    def finalize(self):
        self.speed = 0.0
        self.steer = 0.0
        self.publish_drive_message()
        sys.exit()

def run_operator(make_publisher, publish_callback):
    pygame.init()
    pygame.display.set_mode((100, 100))

    rclpy.init(args=sys.argv)

    args = rclpy.utilities.remove_ros_args(sys.argv[1:])

    max_speed = 8.0
    max_steer = 0.6

    if len(args) >= 1:
        max_speed = float(args[0])
        max_steer = float(args[0])
    
    if len(args) >= 2:
        max_steer = float(args[1])  
        
    keyop_node = CarKeyop('car_keyop', max_speed, max_steer, make_publisher, publish_callback)
    
    try:
        rclpy.spin(keyop_node)
    except KeyboardInterrupt:
        pass
    finally:
        keyop_node.finalize()
        rclpy.shutdown()
