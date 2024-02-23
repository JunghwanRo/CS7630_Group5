#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
import threading

#  Reminder
# Y = G(X)
# Cov(X) = C
#
# Cov(Y) = dG/dX C dG/dX^T
#
# Z = H(X,Y)
# Cov(X) = C
# Cov(Y) = D
#
# Cov(Z) = dG/dX C dG/dX^T + dG/dY D dG/dY^T


class Landmark:
    def __init__(self, Z, X, R):
        # Initialise a landmark based on measurement Z,
        # current position X and uncertainty R
        # TODO
        # L = g(X,Z)
        # Cov(L) = dG/dX Cov(X) dG/dX^T + dG/dZ Cov(Z) dG/dZ^T
        self.L = vstack([0, 0])
        self.P = mat([[0, 0], [0, 0]])

        # L0 = X0 + RTheta*Z
        RTheta = mat([[cos(X[2, 0]), -sin(X[2, 0])], [sin(X[2, 0]), cos(X[2, 0])]])
        self.L = X[0:2] + RTheta * Z

        # Cov(L) = dG/dX Cov(X) dG/dX^T + dG/dZ Cov(Z) dG/dZ^T. However, Cov(X) = 0, so we can ignore the first term
        # dG/dZ = RTheta
        self.P = RTheta * R * RTheta.T
        # TODO END

    def update(self, Z, X, R):
        # Update the landmark based on measurement Z,
        # current position X and uncertainty R
        # TODO
        # Lk = Lk-1 = I2 * Lk-1
        # A = df/dL = I2
        # Q = 1e-12 * I2
        # Z = h(X,L) = RminusTheta * (L - X)
        # dh/dL = RminusTheta

        # Predict
        A = eye(2)
        Q = 1e-12 * eye(2)
        self.L = self.L
        self.P = A * self.P * A.T + Q

        # Update
        theta = X[2, 0]
        Rtheta = mat([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
        R_minus_theta = mat([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])
        # get L with current X and Z
        self.L = X[0:2] + Rtheta * Z
        h_XL = R_minus_theta * (self.L - X[0:2])

        H = R_minus_theta

        # K = kalman gain
        # K = Pk * H^T * inv(H * Pk * H^T + R)
        S = H * self.P * H.T + R
        K = self.P * H.T * inv(S)

        # Lk = Lk-1 + K (Zk - h(Lk-1))
        self.L = self.L + K * (Z - h_XL)

        # Pk = (I - K * H) * Pk
        self.P = (eye(2) - K * H) * self.P
        # TODO END
        return


class MappingKF:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = node.create_publisher(MarkerArray, "~/landmarks", 1)

    def update_ar(self, logger, Z, X, Id, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: Z=" + str(Z.T) + " X=" + str(X.T) + " Id=" + str(Id))
        R = mat(diag([uncertainty, uncertainty]))
        if Id in self.marker_list:
            # Known landmark, we can run the KF update
            # TODO
            self.marker_list[Id].update(Z, X, R)
            logger.info(f"Updated landmark {Id}")
            # TODO END
        else:
            # New landmark, we need to create it
            # TODO
            self.marker_list[Id] = Landmark(Z, X, R)
            logger.info(
                "Initialised landmark %d at %s" % (Id, str(self.marker_list[Id].L.T))
            )
            # TODO END
        self.lock.release()

    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for id in self.marker_list:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0, 0]
            marker.pose.position.y = Lkf.L[1, 0]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 1.0
            marker.pose.orientation.w = 0.0
            marker.scale.x = max(3 * sqrt(Lkf.P[0, 0]), 0.05)
            marker.scale.y = max(3 * sqrt(Lkf.P[1, 1]), 0.05)
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.time.Duration(seconds=3.0).to_msg()
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000 + id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0, 0]
            marker.pose.position.y = Lkf.L[1, 0]
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
