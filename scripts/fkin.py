#!/usr/bin/env python

'''
* fkin.py: Python code to calculate forward kinematics. See the improved version fkin2.py
* Author: Ravi Joshi
* Date: 2016/06/09
'''

import rospy
import PyKDL
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

    #ploting data joint states
    plot_data_s0    = []
    plot_data_s1    = []
    plot_data_w0    = []
    plot_data_w1    = []
    plot_data_w2    = []
    plot_data_e0    = []
    plot_data_e1    = []

    #ploting end-effector states
    plot_data_x     = []
    plot_data_y     = []
    plot_data_z     = []
    plot_data_ang_x = []
    plot_data_ang_y = []
    plot_data_ang_z = []
    plot_data_ang_w = []

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

        #for plotting purpose only
        plot_data_s0.append(joint_state['left_s0'])
        plot_data_s1.append(joint_state['left_s1'])
        plot_data_e0.append(joint_state['left_e0'])
        plot_data_e1.append(joint_state['left_e1'])
        plot_data_w0.append(joint_state['left_w0'])
        plot_data_w1.append(joint_state['left_w1'])
        plot_data_w2.append(joint_state['left_w2'])

        plot_data_x.append(pos[0])
        plot_data_y.append(pos[1])
        plot_data_z.append(pos[2])

        rotation_matrix = PyKDL.Rotation .Quaternion(rot[0],rot[1],rot[2],rot[3])
        [eul_z, eul_y, eul_x] = rotation_matrix.GetEulerZYX ()

        plot_data_ang_x.append(eul_x)
        plot_data_ang_y.append(eul_y)
        plot_data_ang_z.append(eul_z)
#        plot_data_ang_w.append(rot[3])

    savePlots(index, plot_data_s0, plot_data_s1, plot_data_e0, plot_data_e1, plot_data_w0, plot_data_w1, plot_data_w2, plot_data_x, plot_data_y, plot_data_z, plot_data_ang_x, plot_data_ang_y, plot_data_ang_z, plot_data_ang_w)

def savePlots(x, left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2, data_x, data_y, data_z, ang_x, ang_y, ang_z, ang_w):
    plt.figure(1, figsize=(24, 18))
    plt.suptitle('Input for forward kinematics', fontsize=24)

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

    filename = 'data/fkin results/fkin_input.png'
    print 'saving ' + filename
    plt.savefig(filename)

    plt.figure(2, figsize=(24, 18))
    plt.suptitle('Output of forward kinematics using baxter_pykdl', fontsize=24)
    plt.subplot(2, 4, 1)
    plt.plot(x, data_x)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('x (m)')

    plt.subplot(2, 4, 2)
    plt.plot(x, data_y)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('data_y (m)')

    plt.subplot(2, 4, 3)
    plt.plot(x, data_z)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('data_z (m)')

    plt.subplot(2, 4, 4)
    plt.plot(x, ang_x)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('eul_x (radian)')

    plt.subplot(2, 4, 5)
    plt.plot(x, ang_y)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('eul_y (radian)')

    plt.subplot(2, 4, 6)
    plt.plot(x, ang_z)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('eul_z (radian)')

#    plt.subplot(2, 4, 7)
#    plt.plot(x, ang_w)
#    plt.xlabel('time (1unit = 100ms)')
#    plt.ylabel('ang_w (radian)')

    filename = 'data/fkin results/fkin_output_eul.png'
    print 'saving ' + filename
    plt.savefig(filename)

if __name__ == "__main__":
    main()
