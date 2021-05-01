# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

#! /usr/bin/env python
# =============================================================================
# import rospy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# =============================================================================
import time

f  = open("testing.txt", "w+")
for i in range(10):
     f.write("This is line %d\r\n" % (i+4))

# =============================================================================
# if __name__ == '__main__':
# 	listener()
# 
# =============================================================================
