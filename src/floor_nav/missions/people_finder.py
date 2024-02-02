#!/usr/bin/env python3
# ROS specific imports
import random
import sys
import rclpy
from math import pi, radians
from task_manager_client_py.TaskClient import *
from std_srvs.srv import Trigger

rclpy.init(args=sys.argv)
tc = TaskClient("/floor_tasks", 0.2)

scale = 2.0
vel = 0.5

face_received = ConditionVariable("face detected")


def handle_trigger(req, resp):
    global face_received
    face_received.set(True)
    print("face detected")
    resp.success = True
    resp.message = "Done"
    return resp


s = tc.create_service(Trigger, "face_received", handle_trigger)

while True:
    # Wait a bit before start moving
    tc.Wait(duration=1.0)

    face_received.set(False)
    # Start the wait for face task in the background
    w4face = tc.WaitForFace(foreground=False)
    # Prepare a condition so that the following gets executed only until a
    # face is found
    tc.addCondition(ConditionIsCompleted("face detector", tc, w4face))
    tc.addCondition(face_received)

    try:
        tc.Wander()
    except TaskConditionException as e:
        tc.get_logger().info(
            "Path following interrupted on condition: %s"
            % " or ".join([str(c) for c in e.conditions])
        )
        # This means the conditions were triggered. We need to react to it
        tc.StareAtFace()
        tc.Wait(duration=3.0)

        # Turn an angle between 45 to 90 degrees
        angle = 45 + (
            45 * random.random()
        )  # Generates a random angle between 45 and 90
        angle_in_radians = radians(angle)  # Convert the angle to radians
        tc.SetHeading(target=angle_in_radians, relative=True)

tc.get_logger().info("Mission completed")
