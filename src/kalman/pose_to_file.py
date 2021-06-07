#! /usr/bin/env python

import csv
import datetime
import time

import matplotlib.pyplot as plt
import rospy
# from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class SavePoses(object):
    def __init__(self):
        self.delay_before_logging = 2  # in seconds
        self.logging_time = 30  # in seconds
        # Define pose messages
        self._pose_x_true = Pose()
        self._pose_y_true = Pose()
        self._pose_x_noisy = PoseWithCovarianceStamped()
        self._pose_y_noisy = PoseWithCovarianceStamped()
        self._pose_x_filtered = Pose()
        self._pose_y_filtered = Pose()

        self.pose_log_xy = [[], [], [], [], [], []]  # [[true_x], [true_y], [noisy_x], [noisy_y], [filtered_x],
        # [filtered_y]]
        self._pose_sub_true = rospy.Subscriber('/odom', Odometry, self.sub_callback_true)  # true pose subscriber
        self._pose_sub_noisy = rospy.Subscriber('/noisy_pose', PoseWithCovarianceStamped, self.sub_callback_noisy)  # noisy pose
        # subscriber
        self._pose_sub_filtered = rospy.Subscriber('/odometry/filtered', Odometry, self.sub_callback_filtered)
        # filtered pose subscriber

        self.log_pose()  # saves the poses in a giant array
        self.pose_to_plot()  # plots the giant array

    def sub_callback_true(self, msg):  # What will be taken from topic

        self._pose_x_true = msg.pose.pose.position.x
        self._pose_y_true = msg.pose.pose.position.y

    def sub_callback_noisy(self, msg):

        self._pose_x_noisy = msg.pose.pose.position.x
        self._pose_y_noisy = msg.pose.pose.position.y
        # print(msg.pose.pose.position.x)

    def sub_callback_filtered(self, msg):

        self._pose_x_filtered = msg.pose.pose.position.x
        self._pose_y_filtered = msg.pose.pose.position.y

    def log_pose(self):
        # Output: logs of all the trajectory wanted in the plot
        for i in range(self.delay_before_logging):
            rospy.loginfo("Start logging pose in {}".format(i+1))
            time.sleep(1)
        rospy.loginfo("Starts logging")
        # self.poses_dict["pose1"] = self._pose_x
        t_start = time.time()
        t = time.time()
        while t-t_start <= self.logging_time:
            # used try so that if user pressed other than the given key error will not be shown
            self.pose_log_xy[0].append(self._pose_x_true)
            self.pose_log_xy[1].append(self._pose_y_true)
            self.pose_log_xy[2].append(self._pose_x_noisy)
            self.pose_log_xy[3].append(self._pose_y_noisy)
            self.pose_log_xy[4].append(self._pose_x_filtered)
            self.pose_log_xy[5].append(self._pose_y_filtered)

            time.sleep(0.1)
            rospy.loginfo("Remaining time: " + str(int(t-t_start)) + "/" + str(self.logging_time) + "s")
            t = time.time()
        # print(self.pose_log_xy[2])

    def pose_to_plot(self):
        # Output: plot of the xy trajectory of the rover, saved as the current date in svg format
        plt.figure()
        plt.plot(self.pose_log_xy[0], self.pose_log_xy[1], label='True')
        plt.plot(self.pose_log_xy[2], self.pose_log_xy[3], label='Noisy')
        plt.plot(self.pose_log_xy[4], self.pose_log_xy[5], label='Filtered')
        plt.scatter(self.pose_log_xy[0][0], self.pose_log_xy[1][0], label='Start', color='red')
        # plt.scatter(self.pose_log_xy[0][-1], self.pose_log_xy[1][-1], label='End', color='green')

        plt.grid(True)
        plt.legend(loc='best')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show(block=False)

        plt.pause(5)
        rospy.loginfo("Pose plotted")
        plt.savefig("src/kalman/plots/svg/{}.svg".format(str(datetime.datetime.now())))
        plt.savefig("src/kalman/plots/{}.png".format(str(datetime.datetime.now())))
        plt.close()

        rospy.loginfo("Pose saved")

    # def pose_to_csv(self):


if __name__ == "__main__":
    rospy.init_node('pose_plotter', log_level=rospy.INFO)
    try:
        save_spots_object = SavePoses()
    except rospy.ROSInterruptException:
        pass
    # rospy.spin() # maintains the service open.
