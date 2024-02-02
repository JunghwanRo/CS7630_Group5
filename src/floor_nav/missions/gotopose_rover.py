#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient("/floor_tasks", 0.2)

scale = 2.0
vel = 1.0

try:
    tc.GoToPose(goal_x=-scale, goal_y=-scale, goal_theta=-pi / 2, max_velocity=vel)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=-scale, goal_y=scale, goal_theta=pi / 2, max_velocity=vel)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=scale, goal_y=scale, goal_theta=-pi / 2, max_velocity=vel)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=scale, goal_y=-scale, goal_theta=pi / 2, max_velocity=vel)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=-scale, goal_y=-scale, goal_theta=0.0, max_velocity=vel)

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))


tc.get_logger().info("Mission completed")
