#!/usr/bin/env python
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
import numpy
import rclpy
from rclpy.node import Node
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin, atan, tan, fmod
from time import time
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
import math

prefix = ["FL", "FR", "CL", "CR", "RL", "RR"]


def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class RoverMotors:
    def __init__(self):
        self.steering = {}
        self.drive = {}
        self.timestamp = time()
        for k in prefix:
            self.steering[k] = 0.0
            self.drive[k] = 0.0

    def copy(self, value):
        for k in prefix:
            self.steering[k] = value.steering[k]
            self.drive[k] = value.drive[k]
        self.timestamp = value.timestamp


class DriveConfiguration:
    def __init__(self, radius, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self, node):
        self.node = node
        self.X = numpy.asmatrix(numpy.zeros((3, 1)))
        self.motor_state = RoverMotors()
        self.ICR = (pi / 2, pi / 2)
        self.first_run = True
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

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
            return ICR_to_twist(theta, phi, 1e-3)
        elif abs(wz) < 1e-2:
            pass
        else:
            pass
        return twist_out

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False, drive_state=None):
        motors = RoverMotors()
        # This function must return a RoverMotors object, which contains two dictionaries:
        # steering and drive. Steering contains the desired steering angle in radian, and
        # drive the desired motor speed in rad/s. For instance, affecting the rear right wheel
        # speed is done with: motors.drive[“RL”] = 0.5

        # – twist is a standard ros Twist object

        # – drive_cfg is a dictionary of DriveConfiguration object with the following
        # fields: x,y,z: position of the drive in the rover frame, radius of the wheel in
        # meters. For instance, drive_cfg[“FL”].radius is the radius of the front left Wheel.

        # – Skidsteer is a boolean indicating if the rover is operating in skid-steer mode
        # (differential, like a tank) or in the normal mode of rolling without slippage.
        if skidsteer:
            for k in drive_cfg.keys():
                # TODO: In case we are in skidsteer mode (driving like a tank)
                # Insert here the steering and velocity of
                # each wheel in skid-steer mode

                # It is skidsteer mode, so the steering should be 0.0
                motors.steering[k] = 0.0

                # Compute proper drive speed for left wheels, using twist
                # If k ends with "L",
                if k.endswith("L"):
                    motors.drive[k] = (
                        twist.linear.x - twist.angular.z * 1.0
                    ) / drive_cfg[k].radius
                elif k.endswith("R"):
                    motors.drive[k] = (
                        twist.linear.x + twist.angular.z * 1.0
                    ) / drive_cfg[k].radius
        else:
            for k in drive_cfg.keys():
                # TODO: In case we are in rolling without slipping mode (driving normally)
                # Insert here the steering and velocity of
                # each wheel in rolling-without-slipping mode
                W_x = drive_cfg[k].x
                W_y = drive_cfg[k].y
                V_Wx = twist.linear.x - twist.angular.z * W_y
                V_Wy = twist.linear.y + twist.angular.z * W_x
                motors.steering[k] = atan2(V_Wy, V_Wx)
                motors.drive[k] = hypot(V_Wy, V_Wx) / drive_cfg[k].radius
        return motors

    def prepare_inversion_matrix(self, drive_cfg):
        # TODO: Build pseudo inverse of W using the notation from the class. The matrix size below is wrong.
        # Use Least square solution Slide from the Lecture
        """
        START CODE
        iW = numpy.asmatrix(numpy.zeros((2, 3)))
        return iW
        """
        iW = numpy.asmatrix(numpy.zeros((2 * len(drive_cfg), 3)))
        for i, key in enumerate(drive_cfg):
            W_x = drive_cfg[key].x
            W_y = drive_cfg[key].y
            iW[2 * i, :] = [1, 0, -W_y]
            iW[2 * i + 1, :] = [0, 1, W_x]
        # Call the pseudo inverse function from numpy.linalg
        return pinv(iW)

    def prepare_displacement_matrix(self, motor_state_t1, motor_state_t2, drive_cfg):
        # then compute odometry using least square
        # TODO: Build S using the notation from the class. The matrix size below is wrong.
        # Use Least square solution Slide from the Lecture
        """
        START CODE
        S = numpy.asmatrix(numpy.zeros((2, 1)))
        return S
        """
        S = numpy.asmatrix(numpy.zeros((2 * len(drive_cfg), 1)))

        dt = motor_state_t2.timestamp - motor_state_t1.timestamp
        for i, key in enumerate(drive_cfg):
            beta_i = (motor_state_t2.steering[key] + motor_state_t1.steering[key]) / 2
            speed_i = (motor_state_t2.drive[key] + motor_state_t1.drive[key]) / 2
            linear_speed_i = speed_i * drive_cfg[key].radius
            s_i = linear_speed_i * dt
            S[2 * i, 0] = s_i * cos(beta_i)
            S[2 * i + 1, 0] = s_i * sin(beta_i)
        return S

    def compute_displacement(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return numpy.asmatrix(numpy.zeros((3, 1)))
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
        # First compute the local displacement in the robot frame
        dX_local = self.compute_displacement(motor_state, drive_cfg)
        # Debug: print the local displacement
        print(dX_local)
        # TODO: Now integrate the local displacement in the global frame
        """
        START CODE
        self.X[0, 0] += 0.0
        self.X[1, 0] += 0.0
        self.X[2, 0] += 0.0
        return self.X
        """
        to_frame_rel = "world"
        from_frame_rel = "rover_body"
        # Get current transformation from rover frame to world frame
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )
            # Convert Quaternion to Euler angles
            quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            euler = euler_from_quaternion(quat)

            # Only yaw is needed
            yaw = euler[2]

            # Rotate local displacement by yaw angle
            dx_global = dX_local[0] * numpy.cos(yaw) - dX_local[1] * numpy.sin(yaw)
            dy_global = dX_local[0] * numpy.sin(yaw) + dX_local[1] * numpy.cos(yaw)

            # Update X
            self.X[0, 0] += dx_global
            self.X[1, 0] += dy_global
            self.X[2, 0] += dX_local[2]

            # Normalize yaw to [-pi, pi]
            self.X[2, 0] = fmod(self.X[2, 0] + pi, 2 * pi) - pi

        except TransformException as e:
            self.get_logger().error(
                f"Failed to transform from rover_body to world: {str(e)}"
            )
        print(self.X)
        return self.X
