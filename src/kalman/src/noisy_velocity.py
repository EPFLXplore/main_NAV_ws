#!/usr/bin/env python

# ROS stuff
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion

# Standard libraries
import numpy as np
import numpy.linalg as linalg
import math

# Custom values
import rover_geometry as rover
import pose_vel_covariances as cov

""" Directions according to REP-103 (ROS)
    x: forward
    y: left
    z :up
"""


class NoisyVelocity:

    def __init__(self):

        # Geometry of the rover [m]
        self.wheel_radius = 0.105  # meters
        self.R_to_WL1 = None
        self.R_to_WL2 = None
        self.R_to_WL3 = None
        self.R_to_WR1 = None
        self.R_to_WR2 = None
        self.R_to_WR3 = None
        self.R_to_wheels = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Complementary to slip angles [rad]
        self.theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Slip angles [rad]
        self.alpha_WL1 = None
        self.alpha_WL2 = None
        self.alpha_WL3 = None
        self.alpha_WR1 = None
        self.alpha_WR2 = None
        self.alpha_WR3 = None
        self.slip_angles = []

        # Flags
        self.verbose = True
        self.pure_linear = None
        self.pure_rotation = None
        self.combined_motion = None
        self.turn_right = None
        self.turn_left = None

        self.prev_time = 0

        # Ground truth
        self.v_true = np.array([0.0, 0.0, 0.0])
        self.yaw_true = None

        self.v_lin_est = np.array([0.0, 0.0, 0.0])
        self.v_ang_est = 0
        self.velocity_est = TwistWithCovarianceStamped()
        self.yaw_est = 0
        self.wheels_rotational_speed = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad/s
        self.wheels_ang_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad/s
        self.wheels_x_vel_single = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # m/s
        self.wheels_x_vel_vectors = None
        self.wheels_y_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # m/s
        self.wheels_norm_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # m/s norm of each wheel velocity vector
        self.WL1_lin_vel = np.array([0.0, 0.0, 0.0])
        self.WL2_lin_vel = np.array([0.0, 0.0, 0.0])
        self.WL3_lin_vel = np.array([0.0, 0.0, 0.0])
        self.WR1_lin_vel = np.array([0.0, 0.0, 0.0])
        self.WR2_lin_vel = np.array([0.0, 0.0, 0.0])
        self.WR3_lin_vel = np.array([0.0, 0.0, 0.0])
        self.R = np.array([0.0, 0.0, 0.0])  # R[1] = Turn radius = command_lin/command_ang
        self.rate = rospy.Rate(10)
        self.command_sub = rospy.Subscriber("/cmd_vel", Twist, self.command_callback)
        self.joint_sub = rospy.Subscriber('/rover/joint_states', JointState, self.joint_callback, queue_size=10)

        self.pose_sub_true = rospy.Subscriber('/odom', Odometry, self.callback_gd_truth, queue_size=10)  # true

        self.noisy_velocity_msg = rospy.Publisher('/noisy_velocity', TwistWithCovarianceStamped, queue_size=10)
        self.publish_noisy_velocity()

    def callback_gd_truth(self, msg):
        """ Ground truth from ROS """
        self.v_true = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        """ Convert default quaternion to rad angle"""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.yaw_true = euler_from_quaternion(orientation_list)[2]

    def command_callback(self, msg):
        """ Load desired turn radius (hypothesis: it is known)"""
        # Calculates the commanded turn radius based on desired forward and angular velocity
        # print("Fwd vel = " + str(msg.linear.x))
        # print("Ang vel = " + str(msg.angular.z))
        v = msg.linear.x  # Desired linear velocity
        omega = msg.angular.z  # Desired angular velocity
        if omega != 0:
            """ Non-zero rotation speed command """
            self.R[1] = v / omega

            if omega < 0:
                self.turn_right = True
                self.turn_left = False
            elif omega > 0:
                self.turn_right = False
                self.turn_left = True

            if v == 0:
                """ Pure rotation """
                self.pure_rotation = True
                self.pure_linear = False
                self.combined_motion = False

            else:
                """ Combined motion """
                self.pure_rotation = False
                self.pure_linear = False
                self.combined_motion = True
        else:
            """ Pure linear """
            self.R[1] = 100000  # Veeery large R for forward motion

            self.pure_rotation = False
            self.turn_right = False
            self.turn_left = False
            self.pure_linear = True
            self.combined_motion = False

    def joint_callback(self, msg):
        """ Gets measurements from encoders """
        # print("wheel vel " + args[0] + args[1] + " = " + str(msg.process_value) + "\n")
        w_radius = self.wheel_radius
        if len(msg.velocity) != 7:
            return
        else:
            # Forward motion
            self.wheels_rotational_speed = np.array(msg.velocity[-(len(msg.velocity) - 1):])

            self.WL1_lin_vel[0] = w_radius * self.wheels_rotational_speed[0]
            self.WL2_lin_vel[0] = w_radius * self.wheels_rotational_speed[1]
            self.WL3_lin_vel[0] = w_radius * self.wheels_rotational_speed[2]
            self.WR1_lin_vel[0] = w_radius * self.wheels_rotational_speed[3]
            self.WR2_lin_vel[0] = w_radius * self.wheels_rotational_speed[4]
            self.WR3_lin_vel[0] = w_radius * self.wheels_rotational_speed[5]

            self.wheels_x_vel_single = np.array([self.WL1_lin_vel[0], self.WL2_lin_vel[0], self.WL3_lin_vel[0],
                                                 self.WR1_lin_vel[0], self.WR2_lin_vel[0], self.WR3_lin_vel[0]])
            self.wheels_x_vel_vectors = np.array([self.WL1_lin_vel, self.WL2_lin_vel, self.WL3_lin_vel,
                                                  self.WR1_lin_vel, self.WR2_lin_vel, self.WR3_lin_vel])
            # if self.verbose:
            # print("wheels lin vel = " + str(self.wheels_x_vel_single))

    def r_to_wheels(self):
        """ Calculates vector from ICR to center of each wheel """

        # Left side
        self.R_to_WL1 = self.R + rover.RF_to_WL1
        self.R_to_WL2 = self.R + rover.RF_to_WL2
        self.R_to_WL3 = self.R + rover.RF_to_WL3
        # Right side
        self.R_to_WR1 = self.R + rover.RF_to_WR1
        self.R_to_WR2 = self.R + rover.RF_to_WR2
        self.R_to_WR3 = self.R + rover.RF_to_WR3

        self.R_to_wheels = np.array([self.R_to_WL1, self.R_to_WL2, self.R_to_WL3,
                                     self.R_to_WR1, self.R_to_WR2, self.R_to_WR3])

        # if self.verbose:
        #     print("R to WL1 = " + str(self.R_to_wheels[0]))

    def calculate_slip_angles(self):

        """ Slip angles require the wheels to rotate, but for some reason one or several wheels don't rotate the
        corresponding lin_vel will be zero A condition to check whether the wheels are turning or not is then set to
        take the previous value of theta, thus avoiding the program to crash because of a division by 0 """
        if np.any(self.wheels_x_vel_single == 0):

            j1 = np.where(self.wheels_x_vel_single == 0)  # Ouput a tuple with array object in it  = impossible iterate
            j1 = np.asarray(j1)[0]  # transform tuple in simple array = possible to iterate
            for k in j1:
                self.theta[k] = self.theta[
                    k]  # Assign the previous value of theta to the corresponding non-moving wheels

        elif np.any(self.wheels_x_vel_single != 0):
            j2 = np.where(self.wheels_x_vel_single != 0)  # Ouput a tuple with array object in it  = impossible iterate
            j2 = np.asarray(j2)[0]  # transform tuple in simple array = possible to iterate
            for k in j2:
                # print("j2 =  " + str(j2))
                # print("x wheel = " + str(self.wheels_x_vel_vectors))
                # print("R blbl = " + str(self.R_to_wheels))
                self.theta[k] = math.acos(
                    np.dot(self.R_to_wheels[k], self.wheels_x_vel_vectors[k]) / (
                            linalg.norm(self.R_to_wheels[k]) * linalg.norm(self.wheels_x_vel_vectors[k])))
        """
        self.theta[0] = math.acos(
            np.dot(self.R_to_WL1, self.WL1_lin_vel) / (linalg.norm(self.R_to_WL1) * linalg.norm(self.WL1_lin_vel)))
        self.theta[1] = math.acos(
            np.dot(self.R_to_WL2, self.WL2_lin_vel) / (linalg.norm(self.R_to_WL2) * linalg.norm(self.WL2_lin_vel)))
        self.theta[2] = math.acos(
            np.dot(self.R_to_WL3, self.WL3_lin_vel) / (linalg.norm(self.R_to_WL3) * linalg.norm(self.WL3_lin_vel)))
        self.theta[3] = math.acos(
            np.dot(self.R_to_WR1, self.WR1_lin_vel) / (linalg.norm(self.R_to_WR1) * linalg.norm(self.WR1_lin_vel)))
        self.theta[4] = math.acos(
            np.dot(self.R_to_WR2, self.WR2_lin_vel) / (linalg.norm(self.R_to_WR2) * linalg.norm(self.WR2_lin_vel)))
        self.theta[5] = math.acos(
            np.dot(self.R_to_WR3, self.WR3_lin_vel) / (linalg.norm(self.R_to_WR3) * linalg.norm(self.WR3_lin_vel)))
        """
        self.alpha_WL1 = math.pi / 2 - self.theta[0]
        self.alpha_WL2 = math.pi / 2 - self.theta[1]
        self.alpha_WL3 = math.pi / 2 - self.theta[2]
        self.alpha_WR1 = math.pi / 2 - self.theta[3]
        self.alpha_WR2 = math.pi / 2 - self.theta[4]
        self.alpha_WR3 = math.pi / 2 - self.theta[5]

        self.slip_angles = [self.alpha_WL1, self.alpha_WL2, self.alpha_WL3, self.alpha_WR1, self.alpha_WR2,
                            self.alpha_WR3]
        # if self.verbose:
        #     print("alpha WL1 = " + str(math.degrees(self.alpha_WL1)))
        #     print("alpha WR1 = " + str(math.degrees(self.alpha_WR1)))

    def calculate_xy_velocity(self):
        """ Calculate vx, vy in the rover frame (rf) and then transform it to the global frame using the yaw angle
        estimation:
        1) Calculate vx (rf) using the encoders readings
        2) Determine vy (rf) using vx and the desired R (turning radius)
        3) Now the norm of the velocity (rf) can be known and converted to the global frame using the yaw angle"""

        """ Velocity in x direction (forward) """
        left_side = self.wheels_x_vel_single[0:3]
        right_side = self.wheels_x_vel_single[3::]
        # if self.verbose:
        #     print("wheels x vel 1 = " + str(self.wheels_x_vel_single))
        """ The motors for each side are mirrored, so for straight motion their sign is opposite. Need to have them the 
        same to calculate forward velocity """

        if not self.pure_rotation:

            if (left_side > 0).all() and (right_side < 0).all():
                self.wheels_x_vel_single[3::] = -right_side

            if (left_side < 0).all() and (right_side > 0).all():
                self.wheels_x_vel_single[3::] = -right_side

        elif self.pure_rotation:

            if (left_side > 0).all() and (right_side > 0).all():
                self.wheels_x_vel_single[3::] = -right_side

            if (left_side < 0).all() and (right_side < 0).all():
                self.wheels_x_vel_single[3::] = -right_side

        # if self.verbose:
        #     print("wheels x vel 2 = " + str(self.wheels_x_vel_single))
        vx = sum(self.wheels_x_vel_single) / 6  # Average of each motor's forward speed

        """ Velocity in y direction (left) """
        self.r_to_wheels()
        self.calculate_slip_angles()
        for i in range(6):
            self.wheels_y_vel[i] = self.wheels_x_vel_single[i] * np.tan(
                self.slip_angles[i])  # Average of each motor's y speed

        vy = sum(self.wheels_y_vel) / 6

        v_est_rover_frame = math.sqrt(vx ** 2 + vy ** 2)

        """ Take into account the sign of the velocity"""
        if vx >= 0:
            self.v_lin_est[0] = v_est_rover_frame * math.cos(self.yaw_est)
        elif vx < 0:
            self.v_lin_est[0] = -v_est_rover_frame * math.cos(self.yaw_est)

        self.v_lin_est[1] = v_est_rover_frame * math.sin(self.yaw_est)

        self.velocity_est.twist.twist.linear.x = self.v_lin_est[0]
        self.velocity_est.twist.twist.linear.y = self.v_lin_est[1]

    def calculate_angular_velocity(self):
        self.r_to_wheels()
        ang_wheels = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        """ 2 cases:
                    - rover moves forward and turn --> R != 0 --> v = w*R
                    - rover turns on itself (no combined forward motion) --> R = 0 --> can't do v = w*R
        """

        """ Linear velocity of each wheel """

        # if self.combined_motion or self.pure_rotation:
        #
        #     for i in range(len(self.wheels_x_vel_single)):
        #         self.wheels_norm_vel[i] = math.sqrt(self.wheels_x_vel_single[i] ** 2 + self.wheels_y_vel[i] ** 2)
        #         self.wheels_ang_vel[i] = self.wheels_norm_vel[i] / linalg.norm(self.R_to_wheels[i])

        if self.combined_motion:
            r = np.array([0.0, 0.0, 0.0])  # Pass turn radius in 3D for cross product
            r[0:2] = self.R[0:2]
            ang_rover = np.cross(r, self.v_lin_est)  # Cross product of lin vel and R to get angular velocity
            self.v_ang_est = ang_rover[2]

        elif self.pure_rotation:

            self.wheels_ang_vel[0] = \
                np.cross(self.R_to_WL1, np.array([self.wheels_x_vel_single[0], self.wheels_y_vel[0], 0.0]))[2]
            self.wheels_ang_vel[1] = \
                np.cross(self.R_to_WL2, np.array([self.wheels_x_vel_single[1], self.wheels_y_vel[1], 0.0]))[2]
            self.wheels_ang_vel[2] = \
                np.cross(self.R_to_WL3, np.array([self.wheels_x_vel_single[2], self.wheels_y_vel[2], 0.0]))[2]
            self.wheels_ang_vel[3] = \
                np.cross(self.R_to_WR1, np.array([self.wheels_x_vel_single[3], self.wheels_y_vel[3], 0.0]))[2]
            self.wheels_ang_vel[4] = \
                np.cross(self.R_to_WR2, np.array([self.wheels_x_vel_single[4], self.wheels_y_vel[4], 0.0]))[2]
            self.wheels_ang_vel[5] = \
                np.cross(self.R_to_WR3, np.array([self.wheels_x_vel_single[5], self.wheels_y_vel[5], 0.0]))[2]

            self.v_ang_est = sum(self.wheels_ang_vel) / 6

        elif self.pure_linear:
            self.v_ang_est = 0

        if self.turn_right:
            self.v_ang_est = -self.v_ang_est

        self.velocity_est.twist.twist.angular.z = self.v_ang_est

    def calculate_yaw_rover(self):

        now = rospy.get_time()
        dt = now - self.prev_time
        self.yaw_est += self.v_ang_est * dt
        self.prev_time = now

    def encoders_covariances(self):

        self.velocity_est.twist.covariance[0] = cov.cov_vel_x
        self.velocity_est.twist.covariance[7] = cov.cov_vel_y
        self.velocity_est.twist.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity_est.twist.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity_est.twist.covariance[35] = cov.cov_vel_yaw

    def publish_noisy_velocity(self):

        # Loop to publish the noisy odometry values
        self.calculate_yaw_rover()
        self.calculate_xy_velocity()
        self.calculate_angular_velocity()
        self.encoders_covariances()
        self.noisy_velocity_msg.publish(self.velocity_est)

        if self.verbose:
            print("vx_est = " + str(self.velocity_est.twist.twist.linear.x))
            print("vx_true = " + str(self.v_true[0]))
            print("vy_est = " + str(self.velocity_est.twist.twist.linear.y))
            print("vy_true = " + str(self.v_true[1]))
            print("v_ang_est = " + str(self.v_ang_est))
            print("v_ang_true = " + str(self.v_true[2]))
            print("yaw_est = " + str(self.yaw_est))
            print("yaw_true = " + str(self.yaw_true))
            # print("wheels rot speed = " + str(self.wheels_rotational_speed))

            # print("wheels speed = " + str(self.wheels_x_vel_single))


if __name__ == '__main__':
    rospy.init_node('noisy_velocity_node')
    noisy_velocity_object = NoisyVelocity()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            # noisy_pose_object.listen()
            noisy_velocity_object.publish_noisy_velocity()
            rate.sleep()
        except rospy.logwarn("Div by 0 detected"):
            pass
