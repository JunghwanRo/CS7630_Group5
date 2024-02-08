#!/usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin, tan, atan

prefix = ["FL", "FR", "CL", "CR", "RL", "RR"]


class RoverMotors:
    def __init__(self):
        self.steering = {}
        self.drive = {}
        for k in prefix:
            self.steering[k] = 0.0
            self.drive[k] = 0.0

    def copy(self, value):
        for k in prefix:
            self.steering[k] = value.steering[k]
            self.drive[k] = value.drive[k]


class DriveConfiguration:
    def __init__(self, radius, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = np.asmatrix(np.zeros((3, 1)))
        self.motor_state = RoverMotors()
        self.ICR = (pi / 2, pi / 2)
        self.first_run = True

    def ICR_cart_to_polar(x, y):
        return (atan2(y, x), atan(hypot(y, x)))

    def ICR_polar_to_cart(theta, phi):
        r = tan(phi)
        return (r * cos(theta), r * sin(theta))

    def ICR_from_twist(vx, vy, wz):
        theta = atan2(vy, vx) + pi / 2
        phi = atan2(hypot(vx, vy), wz)
        return (theta, phi)

    def ICR_to_twist(theta, phi, v):
        r = tan(phi)
        T = Twist()
        T.angular.z = v / r
        T.linear.x = v * cos(theta - pi / 2)
        T.linear.y = v * sin(theta - pi / 2)
        return T

    def filter_twist(self, twist_in, drive_cfg):
        twist_out = Twist()
        vx = twist_in.linear.x
        vy = twist_in.linear.y
        wz = twist_in.angular.z
        v = hypot(vx, vy)
        if abs(v) < 1e-2:
            # not moving while not changing the current ICR
            theta, phi = self.ICR
            return self.ICR_to_twist(theta, phi, 1e-3)
        elif abs(wz) < 1e-2:
            pass
        else:
            pass
        return twist_out

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False, drive_state=None):
        motors = RoverMotors()
        if skidsteer:
            for k in drive_cfg.keys():
                # First compute the speed from each wheel
                #     V_wheel = V_body + Omega x R
                vw_x = twist.linear.x - 2 * twist.angular.z * drive_cfg[k].y
                vw_y = 0
                motors.steering[k] = 0
                motors.drive[k] = vw_x / drive_cfg[k].radius
        else:
            for k in drive_cfg.keys():
                # First compute the speed from each wheel
                #     V_wheel = V_body + Omega x R
                vw_x = twist.linear.x - twist.angular.z * drive_cfg[k].y
                vw_y = twist.linear.y + twist.angular.z * drive_cfg[k].x
                motors.steering[k] = atan2(vw_y, vw_x)
                motors.drive[k] = hypot(vw_y, vw_x) / drive_cfg[k].radius
                if motors.steering[k] > pi / 2:
                    motors.steering[k] -= pi
                    motors.drive[k] = -motors.drive[k]
                if motors.steering[k] < -pi / 2:
                    motors.steering[k] += pi
                    motors.drive[k] = -motors.drive[k]
                # rospy.loginfo("%s: T %.2f %.2f %.2f V %.2f %.2f S %.2f D %.2f" % \
                #     (k,drive_pose[k]["x"],drive_pose[k]["y"],drive_pose[k]["z"],\
                #     vw_x,vw_y,motors.steering[k]*180./pi,motors.drive[k]))
        return motors

    def prepare_inversion_matrix(self, drive_cfg):
        W = np.asmatrix(np.zeros((len(prefix) * 2, 3)))
        for i in range(len(prefix)):
            k = prefix[i]
            # prepare the least-square matrices
            W[2 * i + 0, 0] = 1
            W[2 * i + 0, 1] = 0
            W[2 * i + 0, 2] = -drive_cfg[k].y
            W[2 * i + 1, 0] = 0
            W[2 * i + 1, 1] = 1
            W[2 * i + 1, 2] = +drive_cfg[k].x
        return pinv(W)

    def prepare_displacement_matrix(self, motor_state_t1, motor_state_t2, drive_cfg):
        # then compute odometry using least square
        S = np.asmatrix(np.zeros((len(prefix) * 2, 1)))
        for i in range(len(prefix)):
            k = prefix[i]
            # compute differentials
            beta = (motor_state_t1.steering[k] + motor_state_t2.steering[k]) / 2
            ds = (motor_state_t2.drive[k] - motor_state_t1.drive[k]) % (2 * pi)
            if ds > pi:
                ds -= 2 * pi
            if ds < -pi:
                ds += 2 * pi
            ds *= drive_cfg[k].radius
            S[2 * i + 0, 0] = ds * cos(beta)
            S[2 * i + 1, 0] = ds * sin(beta)
        return S

    def compute_displacement(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return np.asmatrix(np.zeros((3, 1)))
        # print "-"*32
        # then compute odometry using least square
        # We assume that S = W * dX, hence dX = iW * S, where iW is the pseudo inverse of W
        # TODO: First compute iW based on the wheel positions given in drive_cfg
        iW = self.prepare_inversion_matrix(drive_cfg)
        # TODO: Then compute the displacement matrix S based on the current motor_state, and the previous one stored in self.motor_state
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        # Finally the displacement in the local frame is the product
        dX = iW * S
        # And we backup the current motor state
        self.motor_state.copy(motor_state)
        return dX

    def integrate_odometry(self, motor_state, drive_cfg):
        dX = self.compute_displacement(motor_state, drive_cfg)
        self.X[0, 0] += dX[0, 0] * cos(self.X[2, 0]) - dX[1, 0] * sin(self.X[2, 0])
        self.X[1, 0] += dX[0, 0] * sin(self.X[2, 0]) + dX[1, 0] * cos(self.X[2, 0])
        self.X[2, 0] += dX[2, 0]
        return self.X


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q
