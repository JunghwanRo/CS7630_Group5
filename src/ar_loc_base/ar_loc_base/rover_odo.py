#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos, fmod
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import threading
from rover_driver.rover_kinematics import *


class RoverOdo(RoverKinematics):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__()
        self.first_run = True
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))

    def getRotation(self, theta):
        R = mat(zeros((2, 2)))
        R[0, 0] = cos(theta)
        R[0, 1] = -sin(theta)
        R[1, 0] = sin(theta)
        R[1, 1] = cos(theta)
        return R

    def diffAngle(self, theta1, theta2):
        return fmod(theta1 - theta2 + 4 * pi + pi, 2 * pi) - pi

    def normAngle(self, theta):
        return fmod(theta + 4 * pi + pi, 2 * pi) - pi

    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        self.motor_state.copy(motor_state)

        # Implement Kalman prediction here
        theta = self.X[2, 0]
        Rtheta = mat(
            [[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]]
        )
        # Estimate DeltaX using the pseudo-inverse method
        DeltaX = iW * S
        self.X = self.X + Rtheta * DeltaX
        self.X[2, 0] = self.normAngle(self.X[2, 0])
        self.lock.release()

    def update_ar(self, logger, Z, L, uncertainty):
        # Odometry does not care about AR tags
        return

    def update_compass(self, logger, Z, uncertainty):
        # Odometry does not care about compass
        return

    # Publish the pose for rviz
    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = stamp
        pose.pose.position.x = self.X[0, 0]
        pose.pose.position.y = self.X[1, 0]
        pose.pose.position.z = 0.0
        Q = self.quaternion_from_euler(0, 0, self.X[2, 0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        pose_pub.publish(pose)
        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = child_frame
        odom.pose.pose = pose.pose
        # The remaining field of odometry are not filled for now... maybe one day
        odom_pub.publish(odom)
        return pose

    def broadcast(self, br, target_frame, stamp):
        # Publishing the inverse transform to avoid tree problems in TF
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = stamp
        t.header.frame_id = target_frame
        t.child_frame_id = "arloc"

        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.X[0, 0]
        t.transform.translation.y = self.X[1, 0]
        t.transform.translation.z = 0.0

        q = self.quaternion_from_euler(0, 0, self.X[2, 0])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        br.sendTransform(t)

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci * ck
        cs = ci * sk
        sc = si * ck
        ss = si * sk

        q = empty((4,))
        q[0] = cj * sc - sj * cs
        q[1] = cj * ss + sj * cc
        q[2] = cj * cs - sj * sc
        q[3] = cj * cc + sj * ss

        return q
