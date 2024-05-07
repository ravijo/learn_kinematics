#!/usr/bin/env python

'''
* plot_3d.py: Code for plotting the end-effector postion trajectory from a csv file
* Author: Ravi Joshi
* Date: 2016/06/09
'''

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

EE_Pose_Offset = 1

def readCSV(filename):
    print 'Reading ' + filename + '\n'
    csvData = np.genfromtxt(filename, delimiter=',', names=True)
    csvHeader = ','.join(map(str, csvData.dtype.names))
    csvData = csvData.view(np.float).reshape(csvData.shape + (-1,))
    return csvData

def main(in_file):
    print 'Plotting end-effector trajectory\n'

    # load the data files
    ee_traj  = readCSV(in_file)

    ee_x = ee_traj[:, EE_Pose_Offset + 0] * 1000 # convert meter to mm for visualization only
    ee_y = ee_traj[:, EE_Pose_Offset + 1] * 1000 # convert meter to mm for visualization only
    ee_z = ee_traj[:, EE_Pose_Offset + 2] * 1000 # convert meter to mm for visualization only

    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(ee_x, ee_y, ee_z, label='End-effector trajectory')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'usage: rosrun learn_kinematics plot_data.py in_location out_location\n\tin_file = file containing end-effector trajectory'
    else:
        main(sys.argv[1])
