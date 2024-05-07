#!/usr/bin/env python

'''
* fkin2.py: Python code to calculate forward kinematics. It is an improved version of fkin.py
* Author: Ravi Joshi
* Date: 2016/06/09
'''

import rospy
import argparse
import numpy as np
from os import listdir
import matplotlib.pyplot as plt
from os.path import isfile, join
from baxter_pykdl import baxter_kinematics

LEFT_ANGLE_OFFSET = 1
RIGHT_ANGLE_OFFSET = 8

def fkin(kin_left, kin_right, fileName, saveName):
    # load the data file with header
    data = np.genfromtxt('%s' % (fileName), delimiter=',', names=True)
    header = ','.join(map(str, data.dtype.names))
    data = data.view(np.float).reshape(data.shape + (-1,))

    # get the time stamp for ee file
    time_stamp = data[:,0]

    # creating joint names for input to FK solver
    nJoints = 7
    nSamples = data.shape[0]
    joint_names = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
    left_joints = ['left_' + joint for joint in joint_names]
    right_joints = ['right_' + joint for joint in joint_names]

    pose_var2 = ['x', 'y', 'z', 'x', 'y', 'z', 'w']
    pose_var1 = ['poselinear', 'poselinear', 'poselinear', 'poseangle',
                 'poseangle', 'poseangle', 'poseangle']

    left_pose = [pvar1 + '_left_' + pvar2 for pvar1,pvar2 in zip(pose_var1, pose_var2)]
    right_pose = [pvar1 + '_right_' + pvar2 for pvar1,pvar2 in zip(pose_var1, pose_var2)]
    pose_header = 'time'+','.join(left_pose+right_pose)

    # creating empty array for pose data
    pose_data = np.zeros([nSamples,15], dtype=float)
    pose_data[:,0] = time_stamp

    for i in range(data.shape[0]):
        # defining left and right states
        left_state = dict(zip(left_joints, data[i, LEFT_ANGLE_OFFSET:LEFT_ANGLE_OFFSET+nJoints]))
        right_state = dict(zip(right_joints, data[i, RIGHT_ANGLE_OFFSET:RIGHT_ANGLE_OFFSET+nJoints]))

        # defining left and right poses
        left_pose = np.asmatrix(kin_left.forward_position_kinematics(left_state))
        right_pose = np.asmatrix(kin_right.forward_position_kinematics(right_state))

        # append to pose array
        pose_stamp = np.concatenate((left_pose, right_pose), axis=1)
        pose_data[i,1:] = pose_stamp

    np.savetxt('%s' % (saveName), pose_data, delimiter=',', fmt='%.4f',
               header=pose_header, comments='')

def main():
    # add argument parser
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', '-m', type=int, default=0,
                        help='File process mode (0: File, 1: Dir, Default: 0)')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='Enter filename to be loaded')
    parser.add_argument('--output', '-o', type=str, required=True,
                        help='Enter filename to save EE data')
    args = parser.parse_args()

    # parsing the arguments
    mode = args.mode
    inputVar = args.input
    outputVar = args.output

    # initialize rospy node
    rospy.init_node('baxter_kinematics')
    kin_left = baxter_kinematics('left')
    kin_right = baxter_kinematics('right')

    if mode == 0:
        fkin(kin_left, kin_right, inputVar, outputVar)

    else:
        # get file names
        fileNames = [f for f in listdir(inputVar) if isfile(join(inputVar, f))]

        for fileName in fileNames:
            fkin(kin_left, kin_right, inputVar+fileName, outputVar+fileName)

if __name__ == "__main__":
    main()
