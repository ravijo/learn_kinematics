#!/usr/bin/env python

'''
* plot_data.py: Code for plotting the forwad and inverse kinematics results from a csv file
* Author: Ravi Joshi
* Date: 2016/06/09
'''

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

def readFileMatchingWith(in_location, file_name):
    matched_files = [i for i in os.listdir(in_location) if os.path.isfile(os.path.join(in_location,i)) and file_name in i]
    return readCSV(in_location + matched_files[0]) #read only one file, ignore others if found

def main(in_location, out_location):
    print 'Plot csv files\n'

    # load the data files
    fkin_input       = readFileMatchingWith(in_location, 'fkin_input')
    fkin_kdl_output  = readFileMatchingWith(in_location, 'fkin_kdl_output')
    fkin_trck_output = readFileMatchingWith(in_location, 'fkin_trck_output')
    ikin_kdl_output  = readFileMatchingWith(in_location, 'ikin_kdl_output')
    ikin_trck_output = readFileMatchingWith(in_location, 'ikin_trck_output')

    saveFkinInput(fkin_input, out_location)
    saveFkinCompare(fkin_kdl_output, fkin_trck_output, out_location)
    saveIkinCompare(ikin_kdl_output, ikin_trck_output, out_location)

def readCSV(filename):
    print 'Reading ' + filename + '\n'
    csvData = np.genfromtxt(filename, delimiter=',', names=True)
    csvHeader = ','.join(map(str, csvData.dtype.names))
    csvData = csvData.view(np.float).reshape(csvData.shape + (-1,))
    return csvData

def saveFkinInput(fkin_input, out_location):
    [row, col] = fkin_input.shape
    x = range(0, row)
    left_s0 = fkin_input[:,0]
    left_s1 = fkin_input[:,1]
    left_e0 = fkin_input[:,2]
    left_e1 = fkin_input[:,3]
    left_w0 = fkin_input[:,4]
    left_w1 = fkin_input[:,5]
    left_w2 = fkin_input[:,6]

    plt.figure(1, figsize=(24, 18))
    plt.suptitle('Input for forward kinematics i.e. joint angles', fontsize=24)

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

    filename = out_location + '/fkin_input.png'
    print 'saving ' + filename + '\n'
    plt.savefig(filename)

def saveFkinCompare(fkin_kdl, fkin_trac, out_location):
    [row_kdl, col_kdl] = fkin_kdl.shape
    x_kdl = range(0, row_kdl)
    data_x_kdl = fkin_kdl[:,0]
    data_y_kdl = fkin_kdl[:,1]
    data_z_kdl = fkin_kdl[:,2]
    ang_x_kdl  = fkin_kdl[:,3]
    ang_y_kdl  = fkin_kdl[:,4]
    ang_z_kdl  = fkin_kdl[:,5]
    ang_w_kdl  = fkin_kdl[:,6]

    [row_trac, col_trac] = fkin_trac.shape
    x_trac = range(0, row_trac)

    data_x_trac = fkin_trac[:,0]
    data_y_trac = fkin_trac[:,1]
    data_z_trac = fkin_trac[:,2]
    ang_x_trac  = fkin_trac[:,3]
    ang_y_trac  = fkin_trac[:,4]
    ang_z_trac  = fkin_trac[:,5]
    ang_w_trac  = fkin_trac[:,6]

    plt.figure(2, figsize=(50, 20))
    plt.suptitle('Comparision of output of forward kinematics (top=kdl bottom=trac_ik)', fontsize=24)

    plt.subplot(2, 7, 1)
    plt.plot(x_kdl, data_x_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('x (m)')

    plt.subplot(2, 7, 2)
    plt.plot(x_kdl, data_y_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('y (m)')

    plt.subplot(2, 7, 3)
    plt.plot(x_kdl, data_z_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('z (m)')

    plt.subplot(2, 7, 4)
    plt.plot(x_kdl, ang_x_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_x (radian)')

    plt.subplot(2, 7, 5)
    plt.plot(x_kdl, ang_y_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_y (radian)')

    plt.subplot(2, 7, 6)
    plt.plot(x_kdl, ang_z_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_z (radian)')

    plt.subplot(2, 7, 7)
    plt.plot(x_kdl, ang_w_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_w (radian)')

    plt.subplot(2, 7, 8)
    plt.plot(x_trac, data_x_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('x (m)')

    plt.subplot(2, 7, 9)
    plt.plot(x_trac, data_y_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('y (m)')

    plt.subplot(2, 7, 10)
    plt.plot(x_trac, data_z_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('z (m)')

    plt.subplot(2, 7, 11)
    plt.plot(x_trac, ang_x_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_x (radian)')

    plt.subplot(2, 7, 12)
    plt.plot(x_trac, ang_y_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_y (radian)')

    plt.subplot(2, 7, 13)
    plt.plot(x_trac, ang_z_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_z (radian)')

    plt.subplot(2, 7, 14)
    plt.plot(x_trac, ang_w_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('ang_w (radian)')

    filename = out_location + '/fkin_compare.png'
    print 'saving ' + filename + '\n'
    plt.savefig(filename)

def saveIkinCompare(ikin_kdl, ikin_trac, out_location):
    [row_kdl, col_kdl] = ikin_kdl.shape
    x_kdl     = range(0, row_kdl)
    left_s0_kdl = ikin_kdl[:,0]
    left_s1_kdl = ikin_kdl[:,1]
    left_e0_kdl = ikin_kdl[:,2]
    left_e1_kdl = ikin_kdl[:,3]
    left_w0_kdl = ikin_kdl[:,4]
    left_w1_kdl = ikin_kdl[:,5]
    left_w2_kdl = ikin_kdl[:,6]

    [row_trac, col_trac] = ikin_trac.shape
    x_trac = range(0, row_trac)
    left_s0_trac = ikin_trac[:,0]
    left_s1_trac = ikin_trac[:,1]
    left_e0_trac = ikin_trac[:,2]
    left_e1_trac = ikin_trac[:,3]
    left_w0_trac = ikin_trac[:,4]
    left_w1_trac = ikin_trac[:,5]
    left_w2_trac = ikin_trac[:,6]

    plt.figure(3, figsize=(50, 20))
    plt.suptitle('Comparision of output of inverse kinematics (top=kdl bottom=trac_ik)', fontsize=24)

    plt.subplot(2, 7, 1)
    plt.plot(x_kdl, left_s0_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_s0 (radian)')

    plt.subplot(2, 7, 2)
    plt.plot(x_kdl, left_s1_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_s1 (radian)')

    plt.subplot(2, 7, 3)
    plt.plot(x_kdl, left_e0_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_e0 (radian)')

    plt.subplot(2, 7, 4)
    plt.plot(x_kdl, left_e1_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_e1 (radian)')

    plt.subplot(2, 7, 5)
    plt.plot(x_kdl, left_w0_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w0 (radian)')

    plt.subplot(2, 7, 6)
    plt.plot(x_kdl, left_w1_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w1 (radian)')

    plt.subplot(2, 7, 7)
    plt.plot(x_kdl, left_w2_kdl)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w2 (radian)')

    plt.subplot(2, 7, 8)
    plt.plot(x_trac, left_s0_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_s0 (radian)')

    plt.subplot(2, 7, 9)
    plt.plot(x_trac, left_s1_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_s1 (radian)')

    plt.subplot(2, 7, 10)
    plt.plot(x_trac, left_e0_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_e0 (radian)')

    plt.subplot(2, 7, 11)
    plt.plot(x_trac, left_e1_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_e1 (radian)')

    plt.subplot(2, 7, 12)
    plt.plot(x_trac, left_w0_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w0 (radian)')

    plt.subplot(2, 7, 13)
    plt.plot(x_trac, left_w1_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w1 (radian)')

    plt.subplot(2, 7, 14)
    plt.plot(x_trac, left_w2_trac)
    plt.xlabel('time (1unit = 100ms)')
    plt.ylabel('left_w2 (radian)')

    filename = out_location + '/ikin_compare.png'
    print 'saving ' + filename + '\n'
    plt.savefig(filename)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print 'usage: rosrun learn_kinematics plot_data.py in_location out_location\n\tin_location = directory containing all csv file\n\tout_location = directory to save the generated plots'
    else:
        main(sys.argv[1], sys.argv[2])
