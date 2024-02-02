#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import pygame
from geometry_msgs.msg import Twist

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.publisher_ = self.create_publisher(Twist, '/vrep/twistCommand', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        pygame.event.get()
        twist = Twist()
        # Go Forward/Backward
        twist.linear.x = -joystick.get_axis(1)
        # Turn
        twist.angular.z = joystick.get_axis(0)
        # Fix the turn direction when speed is (-)
        if twist.linear.x > 0.0:
            twist.angular.z = -joystick.get_axis(0)
        self.publisher_.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)

    joy_control = JoyControl()

    rclpy.spin(joy_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
