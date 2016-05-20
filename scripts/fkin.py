#!/usr/bin/python

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
                       'left_w0': eeData[i,3],
                       'left_w1': eeData[i,4],
                       'left_w2': eeData[i,5],
                       'left_e0': eeData[i,6],
                       'left_e1': eeData[i,7]}
        ee_Pose = kin.forward_position_kinematics(joint_state).tolist()
        pos = ee_Pose[:3]
        rot = ee_Pose[3:]

        #for plotting purpose only
        plot_data_s0.append(joint_state['left_s0'])
        plot_data_s1.append(joint_state['left_s1'])
        plot_data_w0.append(joint_state['left_w0'])
        plot_data_w1.append(joint_state['left_w1'])
        plot_data_w2.append(joint_state['left_w2'])
        plot_data_e0.append(joint_state['left_e0'])
        plot_data_e1.append(joint_state['left_e1'])
        plot_data_x.append(pos[0])
        plot_data_y.append(pos[1])
        plot_data_z.append(pos[2])
        plot_data_ang_x.append(rot[0])
        plot_data_ang_y.append(rot[1])
        plot_data_ang_z.append(rot[2])
        plot_data_ang_w.append(rot[3])

    savefig( 1, index,    plot_data_s0,      's0')
    savefig( 2, index,    plot_data_s1,      's1')
    savefig( 3, index,    plot_data_w0,      'w0')
    savefig( 4, index,    plot_data_w1,      'w1')
    savefig( 5, index,    plot_data_w2,      'w2')
    savefig( 6, index,    plot_data_e0,      'e0')
    savefig( 7, index,    plot_data_e1,      'e1')
    savefig( 8, index,     plot_data_x,       'x')
    savefig( 9, index,     plot_data_y,       'y')
    savefig(10, index,     plot_data_z,       'z')
    savefig(11, index, plot_data_ang_x,   'Ang x')
    savefig(12, index, plot_data_ang_y,   'Ang y')
    savefig(13, index, plot_data_ang_z,   'Ang z')
    savefig(14, index, plot_data_ang_w,   'Ang w')

def savefig(num, index, data, filename):
    plt.figure(num)
    plt.plot(index,data)
    plt.title(filename)
    filename = 'data/' + filename + '.png'
    print 'saving ' + filename
    plt.savefig(filename)

if __name__ == "__main__":
    main()
