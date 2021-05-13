#!/usr/bin/env python

import rospy
import tf2_ros
import math
import numpy as np
from std_msgs.msg import String , Float64
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from controller_motors.msg import WheelVelocity


# global variables, dont judge me
pub = rospy.Publisher('/wheel_publisher', WheelVelocity, queue_size=10)

class RoverConstructor:
    def __init__(self):
        # Structure params  [m]
        self.link_1 = 0.2495
        self.link_2 = 0.2895
        self.link_3 = 0.3390
        self.link_4 = 0.1780
        self.link_5 = 0.1495
        self.wheel_r = 0.105    # wheel radius
        self.wheel_t = 0.12     # wheel thickness
        self.wheel_g = 0.013    # gap between wheel and rocker/bogie bars
        self.bar_t = 0.02       # thickness of the rocker and bogie bars
        self.rocker_d = 0.552   # distance between the rocker on both sides
        self.bogie_d = 0.624    # distance between the bogie on both sides
        self.alpha = math.radians(135)        # angle between the suspension links [deg]

        # Motor params  [rpm]
        self.v_max = 6000

rover = RoverConstructor()
command = WheelVelocity()

def compute_velocity(lin_v, ang_v, pots=None):

    # final rotation velocities [rotation_r, rotation_l, rotation_r_mid, rotation_l_mid, rotation_r_rear, rotation_l_rear]
    rotation = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Compute current configuration
    base = np.array([0.0,0.0,0.0])
    rocker_r = np.array([0.0,rover.rocker_d/2,0.0])
    rocker_l = np.array([0.0,-rover.rocker_d/2,0.0])
    bogie_r = np.array([-rover.link_3*math.sin(rover.alpha/2), rover.bogie_d/2, rover.link_3*math.cos(rover.alpha/2)])
    bogie_l = np.array([-rover.link_3*math.sin(rover.alpha/2), -rover.bogie_d/2, rover.link_3*math.cos(rover.alpha/2)])

    wheel_R_1 = np.array([rover.link_2*math.sin(rover.alpha/2)+rover.link_1*math.sin(3*rover.alpha/2-math.pi), rover.rocker_d/2+rover.bar_t/2+rover.wheel_g+rover.wheel_t/2, rover.link_2*math.cos(rover.alpha/2)+rover.link_1*math.cos(3*rover.alpha/2-math.pi)])
    wheel_L_1 = np.array([rover.link_2*math.sin(rover.alpha/2)+rover.link_1*math.sin(3*rover.alpha/2-math.pi), -(rover.rocker_d/2+rover.bar_t/2+rover.wheel_g+rover.wheel_t/2), rover.link_2*math.cos(rover.alpha/2)+rover.link_1*math.cos(3*rover.alpha/2-math.pi)])
    wheel_R_2 = bogie_r + np.array([rover.link_4*math.sin(rover.alpha/2)+rover.link_5*math.sin(3*rover.alpha/2-math.pi), rover.bar_t/2+rover.wheel_g+rover.wheel_t/2, rover.link_4*math.cos(rover.alpha/2)+rover.link_5*math.cos(3*rover.alpha/2-math.pi)])
    wheel_L_2 = bogie_l + np.array([rover.link_4*math.sin(rover.alpha/2)+rover.link_5*math.sin(3*rover.alpha/2-math.pi), -(rover.bar_t/2+rover.wheel_g+rover.wheel_t/2), rover.link_4*math.cos(rover.alpha/2)+rover.link_5*math.cos(3*rover.alpha/2-math.pi)])
    wheel_R_3 = bogie_r + np.array([-(rover.link_4*math.sin(rover.alpha/2)+rover.link_5*math.sin(3*rover.alpha/2-math.pi)), rover.bar_t/2+rover.wheel_g+rover.wheel_t/2, rover.link_4*math.cos(rover.alpha/2)+rover.link_5*math.cos(3*rover.alpha/2-math.pi)])
    wheel_L_3 = bogie_l + np.array([-(rover.link_4*math.sin(rover.alpha/2)+rover.link_5*math.sin(3*rover.alpha/2-math.pi)), -(rover.bar_t/2+rover.wheel_g+rover.wheel_t/2), rover.link_4*math.cos(rover.alpha/2)+rover.link_5*math.cos(3*rover.alpha/2-math.pi)])

    print(wheel_R_1,wheel_R_2,wheel_R_3,wheel_L_1,wheel_L_2,wheel_L_3)

    if ang_v == 0:
        rotation[0] = -lin_v * rover.v_max
        rotation[1] = lin_v * rover.v_max
        rotation[2] = -lin_v * rover.v_max
        rotation[3] = lin_v * rover.v_max
        rotation[4] = -lin_v * rover.v_max
        rotation[5] = lin_v * rover.v_max

    elif lin_v ==0:
        icr = np.array([0.0, lin_v/ang_v, rover.link_4*math.cos(rover.alpha/2)+rover.link_5*math.cos(3*rover.alpha/2-math.pi)])
        # Turning radii of the wheels
        icr_2_wheel_R_1 = np.linalg.norm(icr - wheel_R_1)
        icr_2_wheel_R_2 = np.linalg.norm(icr - wheel_R_2)
        icr_2_wheel_R_3 = np.linalg.norm(icr - wheel_R_3)
        icr_2_wheel_L_1 = np.linalg.norm(icr - wheel_L_1)
        icr_2_wheel_L_2 = np.linalg.norm(icr - wheel_L_2)
        icr_2_wheel_L_3 = np.linalg.norm(icr - wheel_L_3)

        rotation[0] = -icr_2_wheel_R_1
        rotation[1] =  icr_2_wheel_L_1
        rotation[2] = -icr_2_wheel_R_2
        rotation[3] =  icr_2_wheel_L_2
        rotation[4] = -icr_2_wheel_R_3
        rotation[5] =  icr_2_wheel_L_3

        max_ = np.max(rotation)
        rotation = np.multiply(rotation, rover.v_max*ang_v/max_)

    else:
        icr = np.array([0.0, lin_v/ang_v, rover.link_4*math.cos(rover.alpha/2)+rover.link_5*math.cos(3*rover.alpha/2-math.pi)])  # estimated icr of the rover at the hight of the wheel center expressed in the rover frame
        # Turning radii of the wheels
        icr_2_wheel_R_1 = np.linalg.norm(icr - wheel_R_1)
        icr_2_wheel_R_2 = np.linalg.norm(icr - wheel_R_2)
        icr_2_wheel_R_3 = np.linalg.norm(icr - wheel_R_3)
        icr_2_wheel_L_1 = np.linalg.norm(icr - wheel_L_1)
        icr_2_wheel_L_2 = np.linalg.norm(icr - wheel_L_2)
        icr_2_wheel_L_3 = np.linalg.norm(icr - wheel_L_3)
    
        print(icr_2_wheel_R_1, icr_2_wheel_R_2,icr_2_wheel_R_3,icr_2_wheel_L_1,icr_2_wheel_L_2,icr_2_wheel_L_3)

        #scale_factor = np.max([icr_2_wheel_R_1, icr_2_wheel_R_2,icr_2_wheel_R_3,icr_2_wheel_L_1,icr_2_wheel_L_2,icr_2_wheel_L_3])

        # rotation[0] = -icr_2_wheel_R_1*ang_v*rover.v_max/scale_factor
        # rotation[1] = icr_2_wheel_L_1*ang_v*rover.v_max/scale_factor
        # rotation[2] = -icr_2_wheel_R_2*ang_v*rover.v_max/scale_factor
        # rotation[3] = icr_2_wheel_L_2*ang_v*rover.v_max/scale_factor
        # rotation[4] = -icr_2_wheel_R_3*ang_v*rover.v_max/scale_factor
        # rotation[5] = icr_2_wheel_L_3*ang_v*rover.v_max/scale_factor
    
        rotation[0] = -icr_2_wheel_R_1
        rotation[1] =  icr_2_wheel_L_1
        rotation[2] = -icr_2_wheel_R_2
        rotation[3] =  icr_2_wheel_L_2
        rotation[4] = -icr_2_wheel_R_3
        rotation[5] =  icr_2_wheel_L_3

        max_ = np.max(rotation)
        rotation = np.multiply(rotation, rover.v_max*ang_v/max_) + rover.v_max*lin_v
        max_ = np.max(rotation)
        if max_ > rover.v_max:
            rotation = np.multiply(rotation, rover.v_max/max_)

    return rotation


def callback(twist):
    """
    components of Twist() :
    twist.linear.x
    twist.linear.y
    twist.linear.z
    twist.angular.x
    twist.angular.y
    twist.angular.z

    reference rover_base frame axes:
    x positive: forward
    y positive: left
    z positive: up


    """


    lin_vel = twist.linear.x
    ang_vel = twist.angular.z

    # final velocities [rotation_r, rotation_l, rotation_r_mid, rotation_l_mid, rotation_r_rear, rotation_l_rear]
    command_velocity = compute_velocity(lin_vel, ang_vel)

    # compute individual wheel velocity values based on a kinematic model
    # the current kinematic model considers the geometrical distances
    # between the rover base_frame and the wheel frames on flat terrain only
    command.motor_L_1 = command_velocity[1]
    command.motor_L_2 = command_velocity[3]
    command.motor_L_3 = command_velocity[5]
    command.motor_R_1 = command_velocity[0]
    command.motor_R_2 = command_velocity[2]
    command.motor_R_3 = command_velocity[4]
    
    pub.publish( command )
   


def dummy_controller():

    rospy.init_node('twist_to_wheels', anonymous=True)

    # read the tf between suspension joints
    #listener_tf = tf2_ros.TransformListener(tfBuffer)
    # cmd_vel msg contains longitudinal and angular velocities expressed with respect to the base_frame
    sub = rospy.Subscriber("/cmd_vel", Twist, callback)



    rospy.spin()


if __name__ == '__main__':
    try:
        dummy_controller()
    except rospy.ROSInterruptException:
        pass
