#!/usr/bin/env python

# ROS stuff
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Standards libs
import numpy as np
import rospy

# Custom values
import pose_vel_covariances as cov


def callback(msg):
    # save the odometry message and call add_noise() function
    odom_msg = msg
    pub = rospy.Publisher('/noisy_pose', PoseWithCovarianceStamped, queue_size=1)
    noisy_odom_msg = PoseWithCovarianceStamped()
    # Creating the message with the type PoseWithCovarianceStamped
    # filling header with relevant information
    noisy_odom_msg.header.frame_id = "odom"
    noisy_odom_msg.header.stamp = rospy.Time.now()
    # filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose

    # noisy_odom_msg.pose.pose = odom_msg.pose.pose
    noisy_odom_msg.pose.pose.position.x = odom_msg.pose.pose.position.x + np.random.normal(0, cov.noise_var)
    noisy_odom_msg.pose.pose.position.y = odom_msg.pose.pose.position.y + np.random.normal(0, cov.noise_var)
    noisy_odom_msg.pose.pose.position.z = odom_msg.pose.pose.position.z + np.random.normal(0, cov.noise_var)
    #
    noisy_odom_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
    noisy_odom_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
    noisy_odom_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
    noisy_odom_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

    noisy_odom_msg.pose.covariance[0] = cov.noise_var
    noisy_odom_msg.pose.covariance[7] = cov.noise_var
    noisy_odom_msg.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    noisy_odom_msg.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    noisy_odom_msg.pose.covariance[35] = 0

    pub.publish(noisy_odom_msg)


def NoisyPose():
    odom_msg = Odometry()

    noise_var = 0.25
    ctrl_c = False
    rate = rospy.Rate(5)
    # create()
    # loop to publish the noisy odometry values
    while not rospy.is_shutdown():
        odom_subscriber = rospy.Subscriber('/odom', Odometry, callback)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('noisy_pose')
    rospy.loginfo("This node takes the perfect pose from ROS and adds noise through covariance matrix")
    try:
        NoisyPose()
    except rospy.ROSInterruptException:
        pass
