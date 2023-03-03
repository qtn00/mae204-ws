#!/usr/bin/env python
# Script for MAE204 Lab 4: Inverse Kinematics
# Modified by:

import argparse
import numpy as np
from scipy.linalg import expm
from mr_functions import IKinSpace

def wraptopi(angle):
    return (angle+np.pi) % (2*np.pi) - np.pi
def Get_MS():

    # Returns S, list of six screw axes S1...S6, and M, e-e transformation matrix at zero config.

    S1 = np.array([0, 0, 1, -300, 0, 0])

    # ======================= Your Code Starts Here ===============================#
    # Please fill in the correct values for S2~S6, as well as the M matrix
    # Note that M matrix should be different from the one you had for lab 3

    S2 = np.array([0, 1, 0, -240, 0, 0])
    S3 = np.array([0, 1, 0, -240, 0, 244])
    S4 = np.array([0, 1, 0, -240, 0, 457])
    S5 = np.array([0, 0, -1, 169, 457, 0])
    S6 = np.array([0, 1, 0, -155, 0, 457])

    M = np.array([[1, 0, 0, 457], [0, 1, 0, 78], [0, 0, 1, 155], [0, 0, 0, 1]])
    # =============================================================================#

    S = np.array([S1, S2, S3, S4, S5, S6]).T
    print(S)
    return M,S
 
def lab_ik():
    
    # Returns theta_list, list of joint angle sets for each of the predefined waypoints

    M, S = Get_MS()

    # ======================= Your Code Starts Here ===============================#
    # Please calculate inverse kinematics for the 7 given waypoints, and output results in standby...cube2_release joint angle sets (in degrees)
    T1 = np.array([[0, 0,  1,     323.6],
                      [-1, 0,  0,      -335.6],
                      [0, -1, 0, 237],
                      [0, 0,  0,      1]])
    T2 = np.array([[0, 0,  1,     400],
                      [-1, 0,  0, -200],
                      [0, -1, 0, 40],
                      [0, 0,  0,      1]])
    T3 = np.array([[0, 0,  1,     400],
                      [-1, 0,  0, -200],
                      [0, -1, 0, 20],
                      [0, 0,  0,      1]])
    T4 = np.array([[0, 0,  1,     450],
                      [-1, 0,  0, -300],
                      [0, -1, 0, 60],
                      [0, 0,  0,      1]])
    T5 = np.array([[0, 0,  1,     400],
                      [-1, 0,  0, -400],
                      [0, -1, 0, 40],
                      [0, 0,  0,  1]])
    T6 = np.array([[0, 0,  1,     400],
                      [-1, 0,  0, -400],
                      [0, -1, 0, 20],
                      [0, 0,  0,      1]])
    T7 = np.array([[0, 0,  1,     450],
                      [-1, 0,  0, -300],
                      [0, -1, 0, 90],
                      [0, 0,  0,      1]])
    T = np.array([T1,T2,T3,T4,T5,T6,T7])


    thetalist0 = np.array([-2.83703495, -2.4694281,  -0.22527797,  1.12390975, -1.57079633,  0.3045577 ])
    eomg = 0.01
    ev = 0.1


    [standby,f0] = IKinSpace(S,M,T1,thetalist0,eomg,ev)
    #standby = angle_check(standby)
    print(f0)
    [cube1_above,f1] = IKinSpace(S,M,T2,thetalist0,eomg,ev)
    #cube1_above = angle_check(cube1_above)
    print(f1)
    [cube1_grab,f2] = IKinSpace(S,M,T3,thetalist0,eomg,ev)
    #cube1_grab = angle_check(cube1_grab)
    print(f2)
    [cube1_release,f3] = IKinSpace(S,M,T4,thetalist0,eomg,ev)
    #cube1_release = angle_check(cube1_release)
    print(f3)
    [cube2_above,f4] =IKinSpace(S,M,T5,thetalist0,eomg,ev)
    #cube2_above = angle_check(cube2_above)
    print(f4)
    [cube2_grab,f5] = IKinSpace(S,M,T6,thetalist0,eomg,ev)
    #cube2_grab = angle_check(cube2_grab)
    print(f5)
    [cube2_release,f6] = IKinSpace(S,M,T7,thetalist0,eomg,ev)
    #cube2_release = angle_check(cube2_release)
    print(f6)


    # =============================================================================#
    theta_list = np.array([standby, cube1_above, cube1_grab, cube1_release, cube2_above, cube2_grab, cube2_release])
    print(theta_list)
    print(theta_list[1,1])
    for i in range(len(theta_list)):
        for jj in range(len(thetalist0)):
            if theta_list[i,jj] >np.pi or theta_list[i,jj] < np.pi:
                theta_list[i,jj] = wraptopi(theta_list[i,jj]) 


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

  
 

