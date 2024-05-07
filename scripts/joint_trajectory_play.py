#!/usr/bin/env python

'''
* joint_trajectory_play.py: Code to play the joint trajectories from csv file.
*                           See improved version play.py
* Author: Ravi Joshi
* Date: 2016/06/09
'''

import sys
import rospy
import numpy as np
import baxter_interface

Limb_Name = 'left'
Left_S0_Offset = 0 + 0
Left_S1_Offset = 1 + 0
Left_E0_Offset = 2 + 0
Left_E1_Offset = 3 + 0
Left_W0_Offset = 4 + 0
Left_W1_Offset = 5 + 0
Left_W2_Offset = 6 + 0

def main(filename):
    print 'joint trajectory play\n'
    # initialize our ROS node, registering it with the Master
    rospy.init_node('joint_trajectory_play')

    # create an instance of baxter_interface's Limb class
    limb = baxter_interface.Limb(Limb_Name)

    # get the right limb's current joint angles
    angles = limb.joint_angles()

    # load the csv files
    joint_angles = readCSV(filename)

    [row, col] = joint_angles.shape
    for i in range(0, row):
        # assign new joint angles
        angles['left_s0'] = joint_angles[i, Left_S0_Offset]
        angles['left_s1'] = joint_angles[i, Left_S1_Offset]
        angles['left_e0'] = joint_angles[i, Left_E0_Offset]
        angles['left_e1'] = joint_angles[i, Left_E1_Offset]
        angles['left_w0'] = joint_angles[i, Left_W0_Offset]
        angles['left_w1'] = joint_angles[i, Left_W1_Offset]
        angles['left_w2'] = joint_angles[i, Left_W2_Offset]
        print '\ncommanded joint angles '
        print angles
        # move the arm to joint angles
        limb.move_to_joint_positions(angles)

def readCSV(filename):
    print 'Reading ' + filename + '\n'
    csvData = np.genfromtxt(filename, delimiter=',', names=True)
    csvHeader = ','.join(map(str, csvData.dtype.names))
    csvData = csvData.view(np.float16).reshape(csvData.shape + (-1,))
    return csvData

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'usage: rosrun learn_kinematics joint_trajectory_play.py file\n\tfile = csv file containing the left arm joint trajectory\n'
    else:
        main(sys.argv[1])
