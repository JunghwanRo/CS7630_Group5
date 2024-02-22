#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo


class RoverKF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node, initial_pose, initial_uncertainty)
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.ellipse_pub = node.create_publisher(Marker, "~/ellipse", 1)
        self.pose_with_cov_pub = node.create_publisher(
            PoseWithCovarianceStamped, "~/pose_with_covariance", 1
        )

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2, 0])

    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        """
        Modify the predict function to implement the prediction step of the Kalman
        filter. Only modify the part of the function marked with #TODO
        """
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print "-"*32
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        self.motor_state.copy(motor_state)

        # TODO: Implement Kalman prediction here
        # ultimately :
        # self.X =
        # self.P =

        deltaX = iW * S
        dx = deltaX[0]
        dy = deltaX[1]
        dtheta = deltaX[2]

        # Project the state ahead.
        # Xk = f(Xk-1, Uk-1, 0)
        self.X = 
        
        # Project the error covariance ahead.
        # Pk = Ak * Pk-1 * Ak^T + Wk * Qk-1 * Wk^T
        self.P = 

        # TODO END

        self.lock.release()
        return (self.X, self.P)

    def update_ar(self, logger, Z, L, uncertainty):
        """
        Modify the ar_update function to implement the update step of the Kalman
        filter assuming landmarks are observed. The argument Z is a 2x1 vector of the
        measurement in the rover reference frame. L is the position of the observed
        landmark in the world frame as a 2x1 vector. Uncertainty is the uncertainty on the
        measurement process, given as a direction-less radius.
        """
        self.lock.acquire()
        logger.info("Update: L=" + str(L.T) + " X=" + str(self.X.T))
        # TODO
        # self.X =
        # self.P =
        self.lock.release()
        return (self.X, self.P)

    def update_compass(self, logger, Z, uncertainty):
        """
        implement the compass_update function, which updates the state of the
        particles based on a compass (angle only) measurement, and compare the
        performances. Be careful to account for the modulo when computing the
        difference between two angles.
        """
        self.lock.acquire()
        logger.info("Update: S=" + str(Z) + " X=" + str(self.X.T))
        # Implement kalman update using compass here
        # TODO
        # self.X =
        # self.P =
        self.lock.release()
        return (self.X, self.P)

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose_simple = super().publish(
            pose_pub, odom_pub, target_frame, stamp, child_frame
        )
        pose = PoseWithCovarianceStamped()
        pose.header = pose_simple.header
        pose.pose.pose = pose_simple.pose
        C = [0.0] * 36
        C[0] = self.P[0, 0]
        C[1] = self.P[0, 1]
        C[5] = self.P[0, 2]
        C[6] = self.P[1, 0]
        C[7] = self.P[1, 1]
        C[11] = self.P[1, 2]
        C[30] = self.P[2, 0]
        C[31] = self.P[2, 1]
        C[35] = self.P[2, 2]
        pose.pose.covariance = C
        self.pose_with_cov_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        marker.scale.x = 3 * sqrt(self.P[0, 0])
        marker.scale.y = 3 * sqrt(self.P[1, 1])
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.ellipse_pub.publish(marker)
