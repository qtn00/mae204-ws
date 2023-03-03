#!/usr/bin/env python

# The MIT License (MIT)
#
# Copyright (c) 2018-2021 Cristian Beltran
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Cristian Beltran
#
# ------------------------------------------
#
# Modified for UCSD MAE204:Robotics course, Lab 4 : Inverse Kinematics
# Date of modification : 12-21-2022
# Author : Hoseung Seo
# Author email: hoseo@ucsd.edu

import rospy
import numpy as np
import argparse

from ur_control.arm import Arm
from lab4_func import *

def move_joints(joint_angles):
    if not simulate:
        angle_check(np.rad2deg(joint_angles))
    arm.set_joint_positions(position=joint_angles, velocities=None, accelerations=None, wait=True, t=2.0)

def open_gripper(link_name=None):
    if(simulate):
        arm.gripper.open()
        if link_name!=None:
            arm.gripper.release(link_name)
    else:
        gripper_srv(0,5,5)

def close_gripper(link_name=None):
    if(simulate):
        if link_name==None:
            arm.gripper.close()
        else:
            arm.gripper.command(0.04)
            arm.gripper.grab(link_name)
    else:
        gripper_srv(255,5,5)

def lab4_task():
    theta_list = lab_ik()
    standby = theta_list[0]
    cube1_above = theta_list[1]
    cube1_grab = theta_list[2]
    cube1_release = theta_list[3]
    cube2_above = theta_list[4]
    cube2_grab = theta_list[5]
    cube2_release = theta_list[6]

    open_gripper()

    print('Moving to standby position =')
    print(standby)
    move_joints(np.radians(standby))

    print('Moving to cube1_above position =')
    print(cube1_above)
    move_joints(np.radians(cube1_above))

    print('Moving to cube1_grab position =')
    print(cube1_grab)
    move_joints(np.radians(cube1_grab))

    print("Closing gripper...")
    close_gripper(link_name="cube1::link")
    
    print('Moving to standby position =')
    print(standby)
    move_joints(np.radians(standby))

    print('Moving to cube1_release position =')
    print(cube1_release)
    move_joints(np.radians(cube1_release))


    print("Opening gripper...")
    open_gripper(link_name="cube1::link")

    print('Moving to standby position =')
    print(standby)
    move_joints(np.radians(standby))

    print('Moving to cube2_above position =')
    print(cube2_above)
    move_joints(np.radians(cube2_above))

    print('Moving to cube2_grab position =')
    print(cube2_grab)
    move_joints(np.radians(cube2_grab))

    print("Closing gripper...")
    close_gripper(link_name="cube2::link")
    
    print('Moving to standby position =')
    print(standby)
    move_joints(np.radians(standby))

    print('Moving to cube2_release position =')
    print(cube2_release)
    move_joints(np.radians(cube2_release))

    print("Opening gripper...")
    open_gripper(link_name="cube2::link")

    print('Moving to standby position =')
    print(standby)
    move_joints(np.radians(standby))

def main():
    epilog=""
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        formatter_class=arg_fmt, description=main.__doc__, epilog=epilog)
    parser.add_argument(
        '--simulate', action='store_true', help='Use this handle for gazebo simulation')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("lab4_exec", log_level=rospy.INFO)
    global simulate
    simulate = False
    simulate = args.simulate

    ns = ''
    joints_prefix = None
    robot_urdf = "ur3e"
    rospackage = None
    tcp_link = None
    use_gripper = False

    if(simulate):
        use_gripper = True
    else:
        from std_msgs.msg import Int32
        from robotiq_hande_ros_driver.srv import gripper_service
        global gripper_srv
        gripper_srv = rospy.ServiceProxy('gripper_service', gripper_service)

    global arm
    arm = Arm(ft_sensor=False,
              gripper=use_gripper, namespace=ns,
              joint_names_prefix=joints_prefix,
              robot_urdf=robot_urdf, robot_urdf_package=rospackage,
              ee_link=tcp_link)

    lab4_task()
    print("Done.")


if __name__ == '__main__':
    main()
