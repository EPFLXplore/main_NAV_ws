#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def odom_callback(msg):
    # go = Odometry() is not needed
    print("------------------------------------------------")
    # print("pose x = " + str(msg.pose.pose.position.x))
    # print("pose y = " + str(msg.pose.pose.position.y))
    # print("orientation x = " + str(msg.pose.pose.orientation.x))
    # print("orientation y = " + str(msg.pose.pose.orientation.y))
    print("covariance" + str(msg.pose.covariance))


# rate.sleep()


def imu_callback(msg):
    # allez = Imu()
    print("------------------------------------------------")
    print("angular veloc z = " + str(msg.angular_velocity.z))
    print("angular veloc y = " + str(msg.angular_velocity.y))
    print("linear aceleration x = " + str(msg.linear_acceleration.x))
    print("linear acceleration y = " + str(msg.linear_acceleration.y))


# rate.sleep()


def twist(msg):
    # move = Twist()
    print("velocidad linear x = " + str(move.linear.x))
    print("velocidad angular z = " + str(move.angular.z))


# rate.sleep()
# sub=rospy.Subscriber('cmd_vel', Twist, twist)

def listener():
    rospy.init_node('coord_monitor')  # the original name sphero might be the same as other node.
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #topic publisher that allows you to move the sphero
    rate = rospy.Rate(1.5)

    while not rospy.is_shutdown():
        sub_odom = rospy.Subscriber('/noisy_odom', Odometry, odom_callback) # the original name odom might be the same as other function.
        # sub_imu = rospy.Subscriber('/os1_cloud_node/imu', Imu, imu_callback)

        rate.sleep()


if __name__ == '__main__':
    listener()

# while not rospy.is_shutdown():
# move = Twist()
# move.linear.x = 0.1 # m/s. The original value 2 is too large
# move.angular.z= 0.5 # rad/s
# pub.publish(move)
# sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback) # the original name odom might be
# sub_imu = rospy.Subscriber('/os1_cloud_node/imu', Imu, imu_callback)
# rate.sleep() # Instead of using rospy.spin(), we should use rate.sleep because we are in a loop
