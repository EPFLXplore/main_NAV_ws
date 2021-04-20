#!/usr/bin/env python

import rospy
import tf2_ros
from std_msgs.msg import String , Float64
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from numpy import linalg
from controller_motors.msg import WheelVelocity


# global variables, dont judge me
pub = rospy.Publisher('/wheel_publisher', WheelVelocity, queue_size=10) 

command = WheelVelocity()


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
    
    TODO : add saturations at least on twist.angular.z (if not done by another control layer) 
           as too high values (roughly > 4) cause the lift of mid and rear wheels.
           Addtionally, two gain variables are defined to help and their values can be tuned
           (see lines 133-134)
    TODO : add saturations on max min wheel velocity

    """


    # # look at the transformations between the base and wheels frames
    # try:
    #     trans_l_1 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_LEFT_1', rospy.Time(0))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print("Error: tf with left wheel 1 not found")
    #     pass
    # try: 
    #     trans_l_2 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_LEFT_2', rospy.Time(0))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print("Error: tf with left wheel 2 not found")
    #     pass
    # try: 
    #     trans_l_3 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_LEFT_3', rospy.Time(0))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print("Error: tf with left wheel 3 not found")
    #     pass
    # try: 
    #     trans_r_1 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_RIGHT_1', rospy.Time(0))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print("Error: tf with right wheel 1 not found")
    #     pass
    # try: 
    #     trans_r_2 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_RIGHT_2', rospy.Time(0))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print("Error: tf with right wheel 2 not found")
    #     pass
    # try: 
    #     trans_r_3 = tfBuffer.lookup_transform('ROVER_FRAME', 'WHEEL_RIGHT_3', rospy.Time(0))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print("Error: tf with right wheel 3 not found")
    #     pass
    
    # tf_lin_data_fields = [trans_l_1.transform.translation.x, trans_l_1.transform.translation.y, trans_l_1.transform.translation.z]
    # l_b_cpt = linalg.norm(tf_lin_data_fields)
    # tf_lin_data_fields = [trans_r_1.transform.translation.x, trans_r_1.transform.translation.y, trans_r_1.transform.translation.z]
    #m(tf_lin_data_fields)
    # tf_lin_data_fields = [trans_l_2.transform.translation.x, trans_l_2.transform.translation.y, trans_l_2.transform.translation.z]
    # l_mid_b_cpt = linalg.norm(tf_lin_data_fields)
    # tf_lin_data_fields = [trans_r_2.transform.translation.x, trans_r_2.transform.translation.y, trans_r_2.transform.translation.z]
    # r_mid_b_cpt = linalg.norm(tf_lin_data_fields)
    # tf_lin_data_fields = [trans_l_3.transform.translation.x, trans_l_3.transform.translation.y, trans_l_3.transform.translation.z]
    # l_rear_b_cpt = linalg.norm(tf_lin_data_fields)
    # tf_lin_data_fields = [trans_r_3.transform.translation.x, trans_r_3.transform.translation.y, trans_r_3.transform.translation.z]
    # r_rear_b_cpt = linalg.norm(tf_lin_data_fields)


    lin_vel = twist.linear.x
    ang_vel = twist.angular.z


    # compute individual wheel velocity values based on a kinematic model
    # the current kinematic model considers the geometrical distances
    # between the rover base_frame and the wheel-ground contact points only
    if lin_vel == 0 and ang_vel:
        # turn in place
        if ang_vel > 0: 
            # left turn
            rotation_r = - ang_vel
            rotation_l = - ang_vel
            rotation_r_mid = 0 #- ang_vel #*r_mid_b_cpt
            rotation_l_mid = 0 #- ang_vel #*l_mid_b_cpt
            rotation_r_rear = - ang_vel
            rotation_l_rear = - ang_vel
        else: 
            # right turn
            rotation_r = ang_vel
            rotation_l = ang_vel
            rotation_r_mid = 0 #- ang_vel #*r_mid_b_cpt
            rotation_l_mid = 0 #- ang_vel #*l_mid_b_cpt
            rotation_r_rear = ang_vel
            rotation_l_rear = ang_vel
    else:
        # tunable gains to prevent any wheel to lose the contact with the ground
        if ang_vel > 0:                                 
            # left turn
            rotation_r = -lin_vel*0.7 - ang_vel
            rotation_l = lin_vel*0.7 - ang_vel
            rotation_r_mid = -lin_vel*0.7 - ang_vel
            rotation_l_mid = lin_vel*0.7 - ang_vel
            rotation_r_rear = -lin_vel*0.7 - ang_vel
            rotation_l_rear = lin_vel*0.7 - ang_vel
        elif ang_vel < 0:                               
            # right turn
            rotation_r = -lin_vel*0.7 - ang_vel
            rotation_l = lin_vel*0.7 - ang_vel
            rotation_r_mid = -lin_vel*0.7 - ang_vel
            rotation_l_mid = lin_vel*0.7 - ang_vel
            rotation_r_rear = -lin_vel*0.7 - ang_vel
            rotation_l_rear = lin_vel*0.7 - ang_vel
        else:                                                   
            # straight motion
            rotation_r = -lin_vel 
            rotation_l = lin_vel 
            rotation_r_mid = -lin_vel 
            rotation_l_mid = lin_vel 
            rotation_r_rear = -lin_vel 
            rotation_l_rear = lin_vel 


    command.motor_L_1 = rotation_l
    command.motor_L_2 = rotation_l_mid
    command.motor_L_3 = rotation_l_rear
    command.motor_R_1 = rotation_r
    command.motor_R_2 = rotation_r_mid
    command.motor_R_3 = rotation_r_rear
    
    pub.publish( command )
    print(command)
    
    # previous implementation:
    # if twist.angular.z < 0:
    #     rotation_r = -twist.linear.x*2 + twist.angular.z*1
    #     rotation_l = twist.linear.x*2 - twist.angular.z*10

    #     rotation_r_mid = rotation_r*1.4
    #     rotation_l_mid = rotation_l*1
    #     rotation_r_rear = rotation_r*0.5
    #     rotation_l_rear = rotation_l*0.5

    # elif twist.angular.z > 0:
    #     rotation_l = twist.linear.x*2 + twist.angular.z*1
    #     rotation_r = -(twist.linear.x*2 + twist.angular.z*10)

    #     rotation_r_mid = rotation_r*1
    #     rotation_l_mid = rotation_l*1.4
    #     rotation_r_rear = rotation_r*0.5
    #     rotation_l_rear = rotation_l*0.5

    # else:
    #     rotation_r = -twist.linear.x*6
    #     rotation_l = twist.linear.x*6

    #     rotation_r_mid = rotation_r*1
    #     rotation_l_mid = rotation_l*1
    #     rotation_r_rear = rotation_r*1
    #     rotation_l_rear = rotation_l*1


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
