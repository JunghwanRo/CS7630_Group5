#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import inv
from math import pi, sin, cos, hypot, sqrt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo


class MappingKF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node, initial_pose, initial_uncertainty)
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))
        self.P = np.mat(np.diag(initial_uncertainty))
        self.idx = {}
        self.marker_pub = node.create_publisher(MarkerArray, "~/landmarks", 1)

    def predict_rover(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print("-"*32)
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        self.motor_state.copy(motor_state)

        # Implement Kalman prediction here

        # Compute the update in the body frame and the resulting uncertainty in the body frame
        DeltaX = iW * S
        DeltaP = np.zeros((3, 3))

        # jacobians of f
        # theta = self.X[2,0]
        # nb_landmarks = len(self.idx)
        # Jacf_xytheta = np.identity(3) + np.asmatrix([[0, 0, -sin(theta)*DeltaX[0,0] - cos(theta)*DeltaX[1,0]], [0, 0, cos(theta)*DeltaX[0,0] - sin(theta)*DeltaX[1,0]], [0, 0, 0]])
        # A = np.block([[Jacf_xytheta, np.zeros(3,nb_landmarks)],
        #             [np.zeros(nb_landmarks,3), np.identity(nb_landmarks)]])

        # Qr = (encoder_precision**2)*np.identity(3)
        # Q = np.block([[QR, np.zeros(3,nb_landmarks)], [np.zeros(nb_landmarks,3), np.zeros(nb_landmarks,nb_landmarks)]])

        # displacement of the robot
        # already done

        # uncertainty of the robot
        DeltaP = (encoder_precision**2) * np.identity(3)  # not sure about that though

        self.lock.release()
        return self.predict_delta(logger, DeltaX, DeltaP)

    def predict_delta(self, logger, DeltaX, DeltaP):
        self.lock.acquire()
        # Update the state using the provided displacement, but we only need to deal with a subset of the state
        # Assumption: deltaX and deltaP are defined in the body frame and need to be rotated to account for the jacobian
        # of the transfer function
        # TODO
        theta = self.X[2, 0]
        Rtheta = np.mat(
            [[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]]
        )

        # Q = diag([0.1**2, 0.1**2, (0.01)**2]) #covariance on the state X

        self.X[0:3, 0] += Rtheta * DeltaX
        self.P[0:3, 0:3] = Rtheta * DeltaP * np.transpose(Rtheta)

        self.lock.release()
        return (self.X, self.P)

    def update_ar(self, logger, Z, id, uncertainty):
        # Z is a dictionary of id->vstack([x,y])
        self.lock.acquire()
        # TODO
        logger.info("Update: Z=" + str(Z.T) + " X=" + str(self.X.T) + " Id=" + str(id))
        print("Update: Z=" + str(Z.T) + " X=" + str(self.X.T) + " Id=" + str(id))
        # Update the full state self.X and self.P based on landmark id
        # be careful that this might be the first time that id is observed
        # TODO

        theta = self.X[2, 0]
        c = cos(theta)
        s = sin(theta)
        R_minustheta = np.asmatrix([[c, s], [-s, c]])

        if id not in self.idx.keys():  # if first time we see the landmark
            Rtheta = np.transpose(R_minustheta)
            landmark = self.X[0:2, 0] + Rtheta * Z
            self.idx[id] = self.X.shape[0]
            self.X = np.mat(
                np.vstack([self.X, landmark])
            )  # add the landmark to state X
            self.P = np.block(
                [
                    [self.P, np.zeros((self.P.shape[0], 2))],
                    [np.zeros((2, self.P.shape[0])), np.identity(2)],
                ]
            )
            # print(self.P)
            # logger.info("update of P: ",self.P)

        # observation function
        # L = self.X[3+2*id:3+2*(id+1), 0]
        Lid = self.idx[id]
        L = self.X[Lid : Lid + 2, 0]
        h = R_minustheta * (L - self.X[0:2, 0])

        # compute its derivatives with respect to X and L
        lx = L[0, 0]
        ly = L[1, 0]
        x = self.X[0, 0]
        y = self.X[1, 0]

        Jh_X = np.asmatrix(
            [
                [-c, -s, -(lx - x) * s + (ly - y) * c],
                [s, -c, -(lx - x) * c - (ly - y) * s],
            ]
        )

        H = np.block([Jh_X, np.zeros((2, len(self.X) - 3))])
        H[:, Lid : 2 + Lid] = R_minustheta

        # compute kalman gain
        K = self.P * np.transpose(H) * inv(H * self.P * np.transpose(H) + uncertainty)

        self.X += K * (Z - h)
        self.P = (np.identity(np.size(self.P, 0)) - K * H) * self.P
        self.lock.release()
        return (self.X, self.P)

    def update_compass(self, logger, Z, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: S=" + str(Z) + " X=" + str(self.X.T))
        # Update the full state self.X and self.P based on compass measurement
        # TODO
        theta = self.X[2, 0]  # this is our prediction function h here
        h = theta

        # kalman gain K
        # jacobian with respect to X
        H = np.zeros((1, len(self.X)))
        H[0, 2] = 1

        K = self.P * np.transpose(H) * inv(H * self.P * np.transpose(H) + uncertainty)

        self.X += K * (Z - h)
        self.P = (np.identity(np.size(self.P, 0)) - K * H) * self.P
        self.lock.release()
        return (self.X, self.P)

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3 * sqrt(self.P[0, 0])
        marker.scale.y = 3 * sqrt(self.P[1, 1])
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        ma.markers.append(marker)
        for id in self.idx:
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l, 0]
            marker.pose.position.y = self.X[l + 1, 0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 1.0
            marker.pose.orientation.w = 0.0
            marker.scale.x = 3 * sqrt(self.P[l, l])
            marker.scale.y = 3 * sqrt(self.P[l + 1, l + 1])
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.time.Duration(seconds=3.0).to_msg()
            ma.markers.append(marker)
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = 1000 + id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[l + 0, 0]
            marker.pose.position.y = self.X[l + 1, 0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 1.0
            marker.pose.orientation.w = 0.0
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.lifetime = rclpy.time.Duration(seconds=3.0).to_msg()
            ma.markers.append(marker)
        self.marker_pub.publish(ma)
