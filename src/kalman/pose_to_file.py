#! /usr/bin/env python

import csv
import datetime
import time
import numpy as np

import matplotlib.pyplot as plt
import rospy
# from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class SavePoses(object):
    def __init__(self):
        self.delay_before_logging = 2  # in seconds
        self.logging_time = 65  # in seconds
        """ Pose values """
        self._pose_x_true = Pose()
        self._pose_y_true = Pose()
        self._pose_x_noisy = PoseWithCovarianceStamped()
        self._pose_y_noisy = PoseWithCovarianceStamped()
        self._pose_x_filtered = Pose()
        self._pose_y_filtered = Pose()
        self.pose_log_xy = [[], [], [], [], [], []]  # [[true_x], [true_y], [noisy_x], [noisy_y], [filtered_x],
        # [filtered_y]]

        """ Velocities values """
        self._vx_true = 0
        self._vy_true = 0
        self._ang_vel_true = 0
        self._vx_noisy = 0
        self._vy_noisy = 0
        self._ang_vel_noisy = 0
        self._vx_filtered = 0
        self._vy_filtered = 0
        self._ang_vel_filtered = 0
        self.velocity_log = [[], [], [], [], [], [], [], [], []]  # [[true_x], [true_y], [true_ang], [noisy_x],
        # [noisy_y], [noisy_ang], [filtered_x], [filtered_y], [filtered_ang]]

        """ Errors & variances """
        self.err_log = [[], [], [], []]
        self.x_var_meas = 0
        self.y_var_meas = 0
        self.x_var_filtered = 0
        self.y_var_filtered = 0

        self._pose_sub_true = rospy.Subscriber('/odom', Odometry, self.callback_true_pose)  # true pose subscriber
        self._pose_sub_noisy = rospy.Subscriber('/noisy_pose', PoseWithCovarianceStamped, self.callback_noisy_pose)  # noisy pose
        # subscriber
        self._vel_sub_noisy = rospy.Subscriber('/noisy_velocity', PoseWithCovarianceStamped, self.callback_noisy_vel)  # noisy pose
        self._pose_sub_filtered = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_filtered)
        # filtered pose subscriber

        self.log_pose()  # saves the poses in a giant array
        self.pose_to_plot()  # plots the giant array

    def callback_true_pose(self, msg):  # What will be taken from topic

        self._pose_x_true = msg.pose.pose.position.x
        self._pose_y_true = msg.pose.pose.position.y

        self._vx_true = msg.twist.twist.linear.x
        self._vy_true = msg.twist.twist.linear.y
        self._ang_vel_true = msg.twist.twist.angular.z

    def callback_noisy_pose(self, msg):

        self._pose_x_noisy = msg.pose.pose.position.x
        self._pose_y_noisy = msg.pose.pose.position.y
        # print(msg.pose.pose.position.x)

    def callback_noisy_vel(self, msg):
        self._vx_noisy = msg.twist.twist.linear.x
        self._vy_noisy = msg.twist.twist.linear.y
        self._ang_vel_noisy = msg.twist.twist.angular.z

    def callback_filtered(self, msg):

        self._pose_x_filtered = msg.pose.pose.position.x
        self._pose_y_filtered = msg.pose.pose.position.y

        self._vx_filtered = msg.twist.twist.linear.x
        self._vy_filtered = msg.twist.twist.linear.y
        self._ang_vel_filtered = msg.twist.twist.angular.z

    def calculate_variances(self):

        self.x_var_meas = np.var(self.pose_log_xy[2])
        self.y_var_meas = np.var(self.pose_log_xy[3])
        self.x_var_filtered = np.var(self.pose_log_xy[4])
        self.y_var_filtered = np.var(self.pose_log_xy[5])

    def log_pose(self):
        # Output: logs of all the trajectory wanted in the plot
        for i in range(self.delay_before_logging):
            rospy.loginfo("Start logging pose for {} s in {}".format(self.logging_time, i+1))
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

            self.velocity_log[0].append(self._vx_true)
            self.velocity_log[1].append(self._vy_true)
            self.velocity_log[2].append(self._ang_vel_true)
            self.velocity_log[3].append(self._vx_noisy)
            self.velocity_log[4].append(self._vy_noisy)
            self.velocity_log[5].append(self._ang_vel_noisy)
            self.velocity_log[6].append(self._vx_filtered)
            self.velocity_log[7].append(self._vy_true)
            self.velocity_log[8].append(self._ang_vel_true)

            self.err_log[0].append(self._pose_x_true - self._pose_x_noisy)
            self.err_log[1].append(self._pose_x_true - self._pose_x_filtered)
            self.err_log[2].append(self._pose_y_true - self._pose_y_noisy)
            self.err_log[3].append(self._pose_y_true - self._pose_y_filtered)

            time.sleep(0.05)
            rospy.loginfo("Remaining time: {} / {} s".format(int(t-t_start), self.logging_time))
            t = time.time()
        # print(self.pose_log_xy[2])

    def pose_to_plot(self):
        self.calculate_variances()
        # Output: plot of the xy trajectory of the rover, saved as the current date in svg format
        plt.figure("Pose")
        plt.plot(self.pose_log_xy[0], self.pose_log_xy[1], label='True')
        plt.scatter(self.pose_log_xy[2], self.pose_log_xy[3], label='Noisy', marker='.', color='orange')
        plt.scatter(self.pose_log_xy[4], self.pose_log_xy[5], label='Filtered', marker='.', color='green')
        plt.scatter(self.pose_log_xy[0][0], self.pose_log_xy[1][0], label='Start', color='red')

        plt.grid(True)
        plt.legend(loc='best')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('2D pose')
        plt.show(block=False)

        plt.savefig("src/kalman/plots/svg/{}.svg".format(str(datetime.datetime.now())))
        plt.savefig("src/kalman/plots/pdf/{}.pdf".format(str(datetime.datetime.now())))
        plt.savefig("src/kalman/plots/{}.png".format(str(datetime.datetime.now())))

        plt.figure("Err x")
        plt.plot(range(len(self.err_log[0])), self.err_log[0], label="Measured: {}".format(self.x_var_meas))
        plt.plot(range(len(self.err_log[1])), self.err_log[1], label="Filtered: {}".format(self.x_var_filtered))
        plt.grid(True)
        plt.legend(loc='best')
        plt.xlabel('Number of samples')
        plt.ylabel('Error [m]')
        plt.title('Error in x')
        plt.show(block=False)

        plt.savefig("src/kalman/plots/svg/{}.svg".format(str(datetime.datetime.now())))
        plt.savefig("src/kalman/plots/pdf/{}.pdf".format(str(datetime.datetime.now())))
        plt.savefig("src/kalman/plots/{}.png".format(str(datetime.datetime.now())))

        plt.figure("Err y")
        plt.plot(range(len(self.err_log[2])), self.err_log[2], label="Measured: {}".format(self.y_var_meas))
        plt.plot(range(len(self.err_log[3])), self.err_log[3], label="Filtered: {}".format(self.y_var_filtered))
        plt.grid(True)
        plt.legend(loc='best')
        plt.xlabel('Number of samples')
        plt.ylabel('Error [m]')
        plt.title('Error in y')
        plt.show(block=False)

        plt.savefig("src/kalman/plots/svg/{}.svg".format(str(datetime.datetime.now())))
        plt.savefig("src/kalman/plots/pdf/{}.pdf".format(str(datetime.datetime.now())))
        plt.savefig("src/kalman/plots/{}.png".format(str(datetime.datetime.now())))

        plt.pause(5)
        plt.close()

        rospy.loginfo("Pose & errors plotted & saved")


if __name__ == "__main__":
    rospy.init_node('pose_plotter', log_level=rospy.INFO)
    try:
        save_spots_object = SavePoses()
    except rospy.ROSInterruptException:
        pass
    # rospy.spin() # maintains the service open.
