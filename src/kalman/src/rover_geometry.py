#!/usr/bin/env python

import math
import numpy as np

""" MAX 2D"""
RF = np.array([0.0, 0.0, 0.0])
# Left side
RF_to_RL = np.array([0.0, 0.23, 0.0])
RL_to_BL = np.array([-0.31538, 0.046, 0.0])
RL_to_WL1 = np.array([0.38266, 0.12323, 0.0])
BL_to_WL2 = np.array([0.21511, 0.11323, 0.0])
BL_to_WL3 = np.array([-0.25048, 0.11323, 0.0])

RF_to_WL1 = RF_to_RL + RL_to_WL1
RF_to_WL2 = RF_to_RL + RL_to_BL + BL_to_WL2
RF_to_WL3 = RF_to_RL + RL_to_BL + BL_to_WL3
# Right side
RF_to_RR = np.array([0, -0.23, 0])
RR_to_BR = np.array([-0.31538, -0.046, 0.0])
RR_to_WR1 = np.array([0.38266, -0.12323, 0.0])
BR_to_WR2 = np.array([0.24233, -0.11323, 0.0])
BR_to_WR3 = np.array([-0.22434, -0.11323, 0.0])

RF_to_WR1 = RF_to_RR + RR_to_WR1
RF_to_WR2 = RF_to_RR + RR_to_BR + BR_to_WR2
RF_to_WR3 = RF_to_RR + RR_to_BR + BR_to_WR3

""" MAX 3D
RF = np.array([0.0, 0.0, 0.0])
# Left side
RF_to_RL = np.array([0.0, 0.23, 0.0])
RL_to_BL = np.array([-0.31538, 0.046, -0.13063])
RL_to_WL1 = np.array([0.38266, 0.12323, -0.3619])
BL_to_WL2 = np.array([0.21511, 0.11323, -0.24086])
BL_to_WL3 = np.array([-0.25048, 0.11323, -0.20383])

RF_to_WL1 = RF_to_RL + RL_to_WL1
RF_to_WL2 = RF_to_RL + RL_to_BL + BL_to_WL2
RF_to_WL3 = RF_to_RL + RL_to_BL + BL_to_WL3
# Right side
RF_to_RR = np.array([0, -0.23, 0])
RR_to_BR = np.array([-0.31538, -0.046, -0.13063])
RR_to_WR1 = np.array([0.38266, -0.12323, -0.3619])
BR_to_WR2 = np.array([0.24233, -0.11323, -0.21345])
BR_to_WR3 = np.array([-0.22434, -0.11323, -0.23229])

RF_to_WR1 = RF_to_RR + RR_to_WR1
RF_to_WR2 = RF_to_RR + RR_to_BR + BR_to_WR2
RF_to_WR3 = RF_to_RR + RR_to_BR + BR_to_WR3
"""

""" NOT USED """


class RoverConstructor:

    def __init__(self):
        #### MAX ####
        self.RF = np.array([0.0, 0.0, 0.0])
        # Left side
        self.RF_to_RL = np.array([0.0, 0.23, 0.0])
        self.RL_to_BL = np.array([-0.31538, 0.046, -0.13063])
        self.RL_to_WL1 = np.array([0.38266, 0.12323, -0.3619])
        self.BL_to_WL2 = np.array([0.21511, 0.11323, -0.24086])
        self.BL_to_WL3 = np.array([-0.25048, 0.11323, -0.20383])

        self.RF_to_WL1 = self.RF_to_RL + self.RL_to_WL1
        self.RF_to_WL2 = self.RF_to_RL + self.RL_to_BL + self.BL_to_WL2
        self.RF_to_WL3 = self.RF_to_RL + self.RL_to_BL + self.BL_to_WL3
        # Right side
        self.RF_to_RR = np.array([0, -0.23, 0])
        self.RR_to_BR = np.array([-0.31538, -0.046, -0.13063])
        self.RR_to_WR1 = np.array([0.38266, -0.12323, -0.3619])
        self.BR_to_WR2 = np.array([0.24233, -0.11323, -0.21345])
        self.BR_to_WR3 = np.array([-0.22434, -0.11323, -0.23229])

        self.RF_to_WR1 = self.RF_to_RR + self.RR_to_WR1
        self.RF_to_WR2 = self.RF_to_RR + self.RR_to_BR + self.BR_to_WR2
        self.RF_to_WR3 = self.RF_to_RR + self.RR_to_BR + self.BR_to_WR3


        #### SAMUELE AND BASTIEN ####
        # Structure params  [m]
        self.link_1 = 0.2495
        self.link_2 = 0.2895
        self.link_3 = 0.3390
        self.link_4 = 0.1780
        self.link_5 = 0.1495
        self.wheel_r = 0.105  # wheel radius
        self.wheel_t = 0.12  # wheel thickness
        self.wheel_g = 0.013  # gap between wheel and rocker/bogie bars
        self.bar_t = 0.02  # thickness of the rocker and bogie bars
        self.rocker_d = 0.552  # distance between the rocker on both sides
        self.bogie_d = 0.624  # distance between the bogie on both sides
        self.alpha = math.radians(135)  # angle between the suspension links [deg]

        # Motor params  [rpm]
        self.v_max = 6000

        # Compute current configuration
        base = np.array([0.0, 0.0, 0.0])
        rocker_r = np.array([0.0, self.rocker_d / 2, 0.0])
        rocker_l = np.array([0.0, -self.rocker_d / 2, 0.0])
        self.bogie_r = np.array(
            [-self.link_3 * math.sin(self.alpha / 2), self.bogie_d / 2, self.link_3 * math.cos(self.alpha / 2)])
        self.bogie_l = np.array(
            [-self.link_3 * math.sin(self.alpha / 2), -self.bogie_d / 2, self.link_3 * math.cos(self.alpha / 2)])

        self.wheel_R_1 = np.array(
            [self.link_2 * math.sin(self.alpha / 2) + self.link_1 * math.sin(3 * self.alpha / 2 - math.pi),
             self.rocker_d / 2 + self.bar_t / 2 + self.wheel_g + self.wheel_t / 2,
             self.link_2 * math.cos(self.alpha / 2) + self.link_1 * math.cos(3 * self.alpha / 2 - math.pi)])

        self.wheel_L_1 = np.array(
            [self.link_2 * math.sin(self.alpha / 2) + self.link_1 * math.sin(3 * self.alpha / 2 - math.pi),
             -(self.rocker_d / 2 + self.bar_t / 2 + self.wheel_g + self.wheel_t / 2),
             self.link_2 * math.cos(self.alpha / 2) + self.link_1 * math.cos(3 * self.alpha / 2 - math.pi)])

        self.wheel_R_2 = self.bogie_r + np.array(
            [self.link_4 * math.sin(self.alpha / 2) + self.link_5 * math.sin(3 * self.alpha / 2 - math.pi),
             self.bar_t / 2 + self.wheel_g + self.wheel_t / 2,
             self.link_4 * math.cos(self.alpha / 2) + self.link_5 * math.cos(3 * self.alpha / 2 - math.pi)])

        self.wheel_L_2 = self.bogie_l + np.array(
            [self.link_4 * math.sin(self.alpha / 2) + self.link_5 * math.sin(3 * self.alpha / 2 - math.pi),
             -(self.bar_t / 2 + self.wheel_g + self.wheel_t / 2),
             self.link_4 * math.cos(self.alpha / 2) + self.link_5 * math.cos(3 * self.alpha / 2 - math.pi)])

        self.wheel_R_3 = self.bogie_r + np.array(
            [-(self.link_4 * math.sin(self.alpha / 2) + self.link_5 * math.sin(3 * self.alpha / 2 - math.pi)),
             self.bar_t / 2 + self.wheel_g + self.wheel_t / 2,
             self.link_4 * math.cos(self.alpha / 2) + self.link_5 * math.cos(3 * self.alpha / 2 - math.pi)])

        self.wheel_L_3 = self.bogie_l + np.array(
            [-(self.link_4 * math.sin(self.alpha / 2) + self.link_5 * math.sin(3 * self.alpha / 2 - math.pi)),
             -(self.bar_t / 2 + self.wheel_g + self.wheel_t / 2),
             self.link_4 * math.cos(self.alpha / 2) + self.link_5 * math.cos(3 * self.alpha / 2 - math.pi)])

    def print_results(self):
        print("SB wheel l 3 = " + str(self.wheel_L_3))
        print("Moi wheel l 3 = " + str(self.RF_to_WL3))

# if __name__ == '__main__':
#
#     rover = RoverConstructor()
#     rover.print_results()
