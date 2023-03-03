#!/usr/bin/env python
# Script for MAE204 Lab 4: Inverse Kinematics
# Modified by:

import argparse
import numpy as np
from scipy.linalg import expm
from mr_functions import *


def Get_MS():

    # Returns S, list of six screw axes S1...S6, and M, e-e transformation matrix at zero config.

    S1 = np.array([0, 0, 1, -300, 0, 0])

    # ======================= Your Code Starts Here ===============================#
    # Please fill in the correct values for S2~S6, as well as the M matrix
    # Note that M matrix should be different from the one you had for lab 3

    S2 = np.array([0, 0, 0, 0, 0, 0])
    S3 = np.array([0, 0, 0, 0, 0, 0])
    S4 = np.array([0, 0, 0, 0, 0, 0])
    S5 = np.array([0, 0, 0, 0, 0, 0])
    S6 = np.array([0, 0, 0, 0, 0, 0])

    M = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # =============================================================================#

    S = np.array([S1, S2, S3, S4, S5, S6]).T

 
def lab_ik():
    
    # Returns theta_list, list of joint angle sets for each of the predefined waypoints

    M, S = Get_MS()

    # ======================= Your Code Starts Here ===============================#
    # Please calculate inverse kinematics for the 7 given waypoints, and output results in standby...cube2_release joint angle sets (in degrees)






    
    standby = [0, 0, 0, 0, 0, 0]
    cube1_above = [0, 0, 0, 0, 0, 0]
    cube1_grab = [0, 0, 0, 0, 0, 0]
    cube1_release = [0, 0, 0, 0, 0, 0]
    cube2_above = [0, 0, 0, 0, 0, 0]
    cube2_grab = [0, 0, 0, 0, 0, 0]
    cube2_release = [0, 0, 0, 0, 0, 0]


    # =============================================================================#
    theta_list = [standby, cube1_above, cube1_grab, cube1_release, cube2_above, cube2_grab, cube2_release]
    return theta_list

def angle_check(theta_list):
    # Checks if set of joint angles are within joint angle limits
    if theta_list[0] > 180 or theta_list[0] < -180:
        raise ValueError('First joint angle out of bounds')
    if theta_list[1] > 0 or theta_list[1] < -90:
        raise ValueError('Second joint angle out of bounds')
    if theta_list[2] > 180 or theta_list[2] < 0:
        raise ValueError('Third joint angle out of bounds')
    if theta_list[3] > 180 or theta_list[3] < -180:
        raise ValueError('Fourth joint angle out of bounds')
    if theta_list[4] > 180 or theta_list[4] < -180:
        raise ValueError('Fifth joint angle out of bounds')
    if theta_list[5] > 180 or theta_list[5] < -180:
        raise ValueError('Sixth joint angle out of bounds')

def main():
    print("lab4_func.py")

    theta_list = lab_ik()
    print("standby position joint angles =")
    print(theta_list[0])
    print("cube1_above position joint angles =")
    print(theta_list[1])
    print("cube1_grab position joint angles =")
    print(theta_list[2])
    print("cube1_release position joint angles =")
    print(theta_list[3])
    print("cube2_above position joint angles =")
    print(theta_list[4])
    print("cube2_grab position joint angles =")
    print(theta_list[5])
    print("cube2_release position joint angles =")
    print(theta_list[6])
    
    
    print("Done.")

if __name__ == '__main__':
    main()
