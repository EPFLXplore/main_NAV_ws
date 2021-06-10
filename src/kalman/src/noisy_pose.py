#!/usr/bin/env python

# ROS stuff
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Standards libs
import numpy as np
import rospy

# Custom values
import pose_vel_covariances as cov


class NoisyPose:
    def __init__(self):
        self.noisy_pose = PoseWithCovarianceStamped()
        self.odom_msg = Odometry()
        self.noisy_pose_pub = rospy.Publisher('/noisy_pose', PoseWithCovarianceStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.noisy_pose.header.frame_id = "odom"

    def odom_callback(self, msg):
        # save the odometry message and call add_noise() function
        self.odom_msg = msg

    def add_noise(self):
        # Creating the message with the type PoseWithCovarianceStamped
        # filling header with relevant information

        self.noisy_pose.header.stamp = rospy.Time.now()

        # self.noisy_pose.pose.pose = odom_msg.pose.pose
        self.noisy_pose.pose.pose.position.x = self.odom_msg.pose.pose.position.x + np.random.normal(0, cov.noise_xy_var)
        self.noisy_pose.pose.pose.position.y = self.odom_msg.pose.pose.position.y + np.random.normal(0, cov.noise_xy_var)
        self.noisy_pose.pose.pose.position.z = self.odom_msg.pose.pose.position.z + np.random.normal(0, cov.noise_xy_var)
        #
        self.noisy_pose.pose.pose.orientation.x = self.odom_msg.pose.pose.orientation.x
        self.noisy_pose.pose.pose.orientation.y = self.odom_msg.pose.pose.orientation.y
        self.noisy_pose.pose.pose.orientation.z = self.odom_msg.pose.pose.orientation.z
        self.noisy_pose.pose.pose.orientation.w = self.odom_msg.pose.pose.orientation.w

        self.noisy_pose.pose.covariance[0] = cov.noise_xy_var
        self.noisy_pose.pose.covariance[7] = cov.noise_xy_var
        self.noisy_pose.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.noisy_pose.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.noisy_pose.pose.covariance[35] = 0.0

    def publish_noisy_pose(self):
        self.add_noise()
        self.noisy_pose_pub.publish(self.noisy_pose)


if __name__ == '__main__':
    rospy.init_node('noisy_pose_node', anonymous=True)
    noisy_pose_object = NoisyPose()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            noisy_pose_object.publish_noisy_pose()
            rate.sleep()
        except rospy.logwarn("Problem occurred in Noisy Pose"):
            pass
