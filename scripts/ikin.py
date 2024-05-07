#!/usr/bin/env python

'''
* ikin.py: Inverse kinematics calculation using baxter_pykdl
* Author: Ravi Joshi
* Date: 2016/06/09
'''

import rospy
import numpy as np
import matplotlib.pyplot as plt
from baxter_pykdl import baxter_kinematics

def main():
    rospy.init_node('baxter_kinematics')
    print '*** Baxter PyKDL kinematics ***\n'
    kin = baxter_kinematics('left')

    print '\n*** Baxter Description ***\n'
    kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    kin.print_kdl_chain()

    # FK Position
    print '\n*** Baxter Position FK ***\n'

    # load the data files
    eeData = np.genfromtxt('/home/tom/ros_ws/src/learn_kinematics/data/ClothPlayJA', delimiter=',', names=True)
    eeHeader = ','.join(map(str, eeData.dtype.names))
    eeData = eeData.view(np.float).reshape(eeData.shape + (-1,))

    # store the ploting data joint states
    plot_data_s0 = []
    plot_data_s1 = []
    plot_data_w0 = []
    plot_data_w1 = []
    plot_data_w2 = []
    plot_data_e0 = []
    plot_data_e1 = []

    [row, col] = eeData.shape
    index = range(0, row)
    for i in index:
        joint_state = {'left_s0': eeData[i,1],
                       'left_s1': eeData[i,2],
                       'left_e0': eeData[i,3],
                       'left_e1': eeData[i,4],
                       'left_w0': eeData[i,5],
                       'left_w1': eeData[i,6],
                       'left_w2': eeData[i,7]}
        ee_Pose = kin.forward_position_kinematics(joint_state).tolist()

        pos = ee_Pose[:3]
        rot = ee_Pose[3:]
        ikin_output = kin.inverse_kinematics(pos, rot)
        if ikin_output is not None:
            joint_state_ikin = ikin_output.tolist()

        #for plotting purpose only'
        plot_data_s0.append(joint_state_ikin[0])
        plot_data_s1.append(joint_state_ikin[1])
        plot_data_e0.append(joint_state_ikin[2])
        plot_data_e1.append(joint_state_ikin[3])
        plot_data_w0.append(joint_state_ikin[4])
        plot_data_w1.append(joint_state_ikin[5])
        plot_data_w2.append(joint_state_ikin[6])

    savePlots(index, plot_data_s0, plot_data_s1, plot_data_e0, plot_data_e1, plot_data_w0, plot_data_w1, plot_data_w2)

def savePlots(x, left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2):
    plt.figure(1, figsize=(30, 20))
    plt.suptitle('Output of inverse kinematics using baxter_pykdl', fontsize=24)

    plt.subplot(2, 4, 1)
    plt.plot(x, left_s0)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_s0 (radian)')

    plt.subplot(2, 4, 2)
    plt.plot(x, left_s1)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_s1 (radian)')

    plt.subplot(2, 4, 3)
    plt.plot(x, left_e0)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_e0 (radian)')

    plt.subplot(2, 4, 4)
    plt.plot(x, left_e1)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_e1 (radian)')

    plt.subplot(2, 4, 5)
    plt.plot(x, left_w0)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w0 (radian)')

    plt.subplot(2, 4, 6)
    plt.plot(x, left_w1)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w1 (radian)')

    plt.subplot(2, 4, 7)
    plt.plot(x, left_w2)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w2 (radian)')

    filename = '/home/tom/ros_ws/src/learn_kinematics/data/ikin results/ikin_output.png'
    print 'saving ' + filename
    plt.savefig(filename)

if __name__ == "__main__":
    main()
