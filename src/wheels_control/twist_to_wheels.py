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

    rotation_l = Float64()
    rotation_l.data = -10

    rotation_r = Float64()
    rotation_r.data = 10

    pub_r1.publish( rotation_l )
    pub_r2.publish( rotation_l )
    pub_r3.publish( rotation_l )

    pub_l1.publish( rotation_r )
    pub_l2.publish( rotation_r )
    pub_l3.publish( rotation_r )



def dummy_controller():



    rospy.init_node('twist_to_wheels', anonymous=True)

    sub = rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()



if __name__ == '__main__':
    try:
        dummy_controller()
    except rospy.ROSInterruptException:
        pass