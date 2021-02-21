#!/usr/bin/env python

import rospy
from std_msgs.msg import String , Float64

from geometry_msgs.msg import Twist



# global variables, dont judge me
pub_r1 = rospy.Publisher('/rover/right_joint_effort_controller_1/command', Float64, queue_size=10)
pub_r2 = rospy.Publisher('/rover/right_joint_effort_controller_2/command', Float64, queue_size=10)
pub_r3 = rospy.Publisher('/rover/right_joint_effort_controller_3/command', Float64, queue_size=10)

pub_l1 = rospy.Publisher('/rover/left_joint_effort_controller_1/command', Float64, queue_size=10)
pub_l2 = rospy.Publisher('/rover/left_joint_effort_controller_2/command', Float64, queue_size=10)
pub_l3 = rospy.Publisher('/rover/left_joint_effort_controller_3/command', Float64, queue_size=10)

rotation_l = Float64()
rotation_l.data = 0
rotation_l_mid = Float64()
rotation_l_mid.data = 0
rotation_l_rear = Float64()
rotation_l_rear.data = 0

rotation_r = Float64()
rotation_r.data = 0
rotation_r_mid = Float64()
rotation_r_mid.data = 0
rotation_r_rear = Float64()
rotation_r_rear.data = 0


def callback(twist):

    rospy.loginfo(twist)

    ''' components of Twist() :
    twist.linear.x
    twist.linear.y
    twist.linear.z
    twist.angular.x
    twist.angular.y
    twist.angular.z
    '''

    #definition (view from rover): x positive: forward, y positive: left, z positive: up

    if twist.angular.z < 0:
        rotation_r.data = -twist.linear.x*2 + twist.angular.z*1
        rotation_l.data = twist.linear.x*2 - twist.angular.z*10

        rotation_r_mid.data = rotation_r.data*1.4
        rotation_l_mid.data = rotation_l.data*1
        rotation_r_rear.data = rotation_r.data*0.5
        rotation_l_rear.data = rotation_l.data*0.5

    elif twist.angular.z > 0:
        rotation_l.data = twist.linear.x*2 - twist.angular.z*1
        rotation_r.data = -(twist.linear.x*2 + twist.angular.z*10)

        rotation_r_mid.data = rotation_r.data*1
        rotation_l_mid.data = rotation_l.data*1.4
        rotation_r_rear.data = rotation_r.data*0.5
        rotation_l_rear.data = rotation_l.data*0.5

    else:
        rotation_r.data = -twist.linear.x*6
        rotation_l.data = twist.linear.x*6

        rotation_r_mid.data = rotation_r.data*1
        rotation_l_mid.data = rotation_l.data*1
        rotation_r_rear.data = rotation_r.data*1
        rotation_l_rear.data = rotation_l.data*1




    pub_r1.publish( rotation_r )
    pub_r2.publish( rotation_r_mid )
    pub_r3.publish( rotation_r_rear )

    pub_l1.publish( rotation_l )
    pub_l2.publish( rotation_l_mid )
    pub_l3.publish( rotation_l_rear )



def dummy_controller():



    rospy.init_node('twist_to_wheels', anonymous=True)

    sub = rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()



if __name__ == '__main__':
    try:
        dummy_controller()
    except rospy.ROSInterruptException:
        pass
