#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg       import Float64
from tf.transformations import euler_from_quaternion


#def get_rotation():
    # get rotation between the frames in Euler angles
#    tf_rotation_data_fields = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
#    (roll, pitch, yaw) = euler_from_quaternion(tf_rotation_data_fields)
#    return pitch


if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    #turtle_name = rospy.get_param('turtle', 'turtle2')
    #spawner(4, 2, 0, turtle_name)

    pub_base_angle_correction = rospy.Publisher('/rover/transmission_rocker_left/command', Float64, queue_size=10)

    #rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        '''
        try: # we look for the transformation from the latest -the 0 in rospy.Time(0)- CHASSIS to the ROVER_BASE frame
            trans_1 = tfBuffer.lookup_transform('CHASSIS', 'ROVER_BASE', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue'''
        try: # we look for the transformation from the latest -the 0 in rospy.Time(0)- CHASSIS to the ROVER_BASE frame
            trans_2 = tfBuffer.lookup_transform('Rover_frame', 'ROCKER_LEFT', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        '''
        try: # we look for the transformation from the latest -the 0 in rospy.Time(0)- CHASSIS to the ROVER_BASE frame
            trans_3 = tfBuffer.lookup_transform('ROVER_BASE', 'ROCKER_RIGHT', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue'''


        # get rotation between the frames in Euler angles
        #tf_rotation_data_fields = [trans_1.transform.rotation.x, trans_1.transform.rotation.y, trans_1.transform.rotation.z, trans_1.transform.rotation.w]
        #(roll_1, pitch_1, yaw_1) = euler_from_quaternion(tf_rotation_data_fields)
        tf_rotation_data_fields = [trans_2.transform.rotation.x, trans_2.transform.rotation.y, trans_2.transform.rotation.z, trans_2.transform.rotation.w]
        (roll_2, pitch_2, yaw_2) = euler_from_quaternion(tf_rotation_data_fields)
        #tf_rotation_data_fields = [trans_3.transform.rotation.x, trans_3.transform.rotation.y, trans_3.transform.rotation.z, trans_3.transform.rotation.w]
        #(roll_3, pitch_3, yaw_3) = euler_from_quaternion(tf_rotation_data_fields)
        
        #roll, pitch and yaw are of Float type and they are [rad]

        err_angle = Float64()
        err_angle.data = 0
        #err_angle.data = - (pitch_1)
        p_gain = 0.5   #to be tuned
        ctrl_cmd = (err_angle.data) 

        print(ctrl_cmd)
        pub_base_angle_correction.publish(ctrl_cmd)

        #print("pitch 1", pitch_1, "pitch 2", pitch_2, "pitch 3", pitch_3, "commanded action", ctrl_cmd)


        #msg = geometry_msgs.msg.Twist()

        #msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        #msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        #turtle_vel.publish(msg)

        rate.sleep()