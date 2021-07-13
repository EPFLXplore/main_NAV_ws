#!/usr/bin/env python
"""This file is here to ease the covariance tuning of the data sources (LIDAR, encoders)"""

""" Covariance noise for LIDAR data (xy pose)"""

noise_xy_var = 0.5  # m
noise_yaw_var = 0.01  # rad

""" Covariance noise for encoders (odometry) data"""

cov_vel_x = 0.5  # m/s
cov_vel_y = 0.5  # m/s
cov_vel_yaw = 0.1  # rad/s
