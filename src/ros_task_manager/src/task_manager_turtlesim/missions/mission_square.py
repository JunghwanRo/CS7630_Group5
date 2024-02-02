#!/usr/bin/python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/turtlesim_tasks', 0.2)

wp = [ [1., 9., pi/2, 0, 0, 255],
    [9., 9., 0., 0, 255, 255],
    [9., 1., -pi/2, 0, 255, 0],
    [1., 1., -pi, 255, 255, 0]]

while True:
    tc.Clear()

    tc.Wait(duration=1.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=1.0,goal_y=1.0)

    for p in wp:
        tc.Wait(duration=0.2)
        tc.ReachAngle(target=p[2])
        tc.SetPen(on=True,r=p[3],g=p[4],b=p[5])
        tc.GoTo(goal_x=p[0],goal_y=p[1])

    tc.Wait(duration=2.)
    tc.SetPen(on=False)
    tc.GoTo(goal_x=5.0,goal_y=5.0)
    tc.ReachAngle(target=pi/2)

tc.get_logger().info("Mission completed")


