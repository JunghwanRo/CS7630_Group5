#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import bisect
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo


class RoverPF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node, initial_pose, initial_uncertainty)
        self.N = 500
        self.particles = [
            self.X + self.drawNoise(initial_uncertainty) for i in range(0, self.N)
        ]
        self.pa_pub = node.create_publisher(PoseArray, "~/particles", 1)
        # print self.particles

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2, 0])

    def drawNoise(self, norm):
        if type(norm) == list:
            return mat(vstack(norm) * (2 * random.rand(3, 1) - vstack([1, 1, 1])))
        else:
            return mat(multiply(norm, ((2 * random.rand(3, 1) - vstack([1, 1, 1])))))

    def applyDisplacement(self, X, DeltaX, Uncertainty):
        # TODO: apply the displacement DeltaX, in the robot frame, to the particle X expressed in the world frame,
        # including the uncertainty present in variable uncertainty
        return X

    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return self.X
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        self.motor_state.copy(motor_state)

        # Apply the particle filter prediction step here
        """
        Modify the predict function to implement the prediction step of the particle
        filter. Note that the particles are stored in self.particles. Only modify the part of
        the function marked with Todo"""
        # TODO
        # DeltaX = iW*S
        # Note, using the function applyDisplacement could be useful to compute the new particles
        # self.particles = ...
        DeltaX = iW * S
        for i in range(self.N):
            self.particles[i] = self.applyDisplacement(
                self.particles[i], DeltaX, encoder_precision
            )

        # TODO END
        self.updateMean()
        self.lock.release()

    def evalParticleAR(self, X, Z, L, Uncertainty):
        # Returns the fitness of a particle with state X given observation Z of landmark L
        return 0

    def evalParticleCompass(self, X, Value, Uncertainty):
        # Returns the fitness of a particle with state X given compass observation value
        # Beware of the module when computing the difference of angles
        return 0

    def update_ar(self, logger, Z, L, Uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: L=" + str(L.T) + " X=" + str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleAR could be useful
        """Modify the ar_update function to implement the update step of the particle
        filter assuming landmarks are observed. The argument Z is a 2x1 vector of the
        measurement in the rover reference frame. L is the position of the observed
        landmark in the world frame as a 2x1 vector. Uncertainty is the uncertainty on the
        measurement process, given as a direction-less radius."""

        # TODO
        # self.particles = ...
        weights = numpy.zeros(self.N)
        for i in range(self.N):
            weights[i] = self.evalParticleAR(self.particles[i], Z, L, Uncertainty)
        weights /= sum(weights)  # Normalize weights
        # Resampling step might be needed here based on the weights
        # TODO END

        self.updateMean()
        self.lock.release()

    def update_compass(self, logger, angle, Uncertainty):
        self.lock.acquire()
        # TODO
        # print self.particles
        logger.info("Update: S=" + str(angle) + " X=" + str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleCompass could be useful
        """implement the compass_update function, which updates the state of the
        particles based on a compass (angle only) measurement, and compare the
        performances. Be careful to account for the modulo when computing the
        difference between two angles."""

        # self.particles = ...

        self.updateMean()
        self.lock.release()

    def updateMean(self):
        X = mat(zeros((3, 1)))
        for x in self.particles:
            X += x
        self.X = X / len(self.particles)

        return self.X

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)

        pa = PoseArray()
        pa.header = pose.header
        for p in self.particles:
            po = Pose()
            po.position.x = p[0, 0]
            po.position.y = p[1, 0]
            q = self.quaternion_from_euler(0, 0, p[2, 0])
            po.orientation.x = q[0]
            po.orientation.y = q[1]
            po.orientation.z = q[2]
            po.orientation.w = q[3]
            pa.poses.append(po)
        self.pa_pub.publish(pa)