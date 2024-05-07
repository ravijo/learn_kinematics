#!/usr/bin/env python

'''
* convert.py: This file merge data_file to dummy_file and produce out_file
* Author: Ravi Joshi
* Date: 2016/06/09
* Description: The compare_ik proragm calculate inverse kinematics only for left arm.
*              In order to verify the results, we want to play the trajectory in simulator.
*              The simulator needs complete file, which must have joint angles for
*              both the arms. Thus this program is used to create complete file by
*              replacing the lines in dummy_file by data_file
'''

import sys
import numpy as np

Left_S0_Offset = 1
Left_S1_Offset = 2
Left_E0_Offset = 3
Left_E1_Offset = 4
Left_W0_Offset = 5
Left_W1_Offset = 6
Left_W2_Offset = 7
CSV_Header = 'time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2'

def main(dummy_file, data_file, out_file):
    print 'convert csv file\n'

    # load the csv files
    dummy_data = readCSV(dummy_file)
    actual_data = readCSV(data_file)

    [row, col] = actual_data.shape

    dummy_data[0:row, Left_S0_Offset] = actual_data[:, Left_S0_Offset - 1]
    dummy_data[0:row, Left_S1_Offset] = actual_data[:, Left_S1_Offset - 1]
    dummy_data[0:row, Left_E0_Offset] = actual_data[:, Left_E0_Offset - 1]
    dummy_data[0:row, Left_E1_Offset] = actual_data[:, Left_E1_Offset - 1]
    dummy_data[0:row, Left_W0_Offset] = actual_data[:, Left_W0_Offset - 1]
    dummy_data[0:row, Left_W1_Offset] = actual_data[:, Left_W1_Offset - 1]
    dummy_data[0:row, Left_W2_Offset] = actual_data[:, Left_W2_Offset - 1]

    # we need to remove the remaing rows of csv
    [row_dummy, col] = dummy_data.shape
    if row_dummy > row:
        np.delete(dummy_data, np.s_[row:], 0)

    writeCSV(out_file, dummy_data)

def readCSV(filename):
    print 'Reading ' + filename
    csvData = np.genfromtxt(filename, delimiter=',', names=True)
    csvHeader = ','.join(map(str, csvData.dtype.names))
    csvData = csvData.view(np.float).reshape(csvData.shape + (-1, ))
    return csvData

def writeCSV(filename, data):
    print 'Writing ' + filename + '\n'
    np.savetxt(filename, data, header=CSV_Header, delimiter=",")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print '''usage: rosrun learn_kinematics convert.py dummy_file data_file out_file
\tdummy_file = file recorded using baxter recorder program, which contains all data as csv format
\tdata_file = data file containing only the joint angles of left arm
\tout_file = output file name'''
    else:
        main(sys.argv[1], sys.argv[2], sys.argv[3])
