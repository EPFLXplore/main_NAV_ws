#!/usr/bin/env python
"""This file is here to ease the covariance tuning of the data sources (LIDAR, encoders)"""

""" Covariance noise for LIDAR data (xy pose)"""

noise_var = 0.05  # m

""" Covariance noise for encoders (odometry) data"""

cov_vel_x = 1  # m/s
cov_vel_y = 1  # m/s
cov_vel_yaw = 1  # rad/s
