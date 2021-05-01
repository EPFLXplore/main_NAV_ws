#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import numpy as np


# Adds Gaussian noise to the (almost) perfect ROS odometry #

class lidar_Odom:

    def __init__(self):
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.odom_publisher = rospy.Publisher('/lidar_odom', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(5)

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.ctrl_c = True

    def odom_callback(self, msg):
        # save the odometry message and call add_noise() function
        self.odom_msg = msg
        self.add_noise()

    def add_noise(self):
        # add noise to the Y position value of the odometry message
        noise_var = 0.01
        noise_mean = 0
        rand_float_x = np.random.normal(noise_mean, noise_var)
        rand_float_y = np.random.normal(noise_mean, noise_var)
        self.odom_msg.pose.pose.position.y = self.odom_msg.pose.pose.position.y + rand_float_x
        self.odom_msg.pose.pose.position.x = self.odom_msg.pose.pose.position.x + rand_float_y

    def publish_lidar_odom(self):
        # loop to publish the noisy odometry values
        while not rospy.is_shutdown():
            self.odom_publisher.publish(self.odom_msg)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('lidar_odom_node', anonymous=True)
    lidar_odom_object = lidar_Odom()
    try:
        lidar_odom_object.publish_lidar_odom()
    except rospy.ROSInterruptException:
        pass
