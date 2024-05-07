#!/usr/bin/env python

'''
* plot_performance.py: Code for plotting the kdl vs trac_ik performance
* Author: Ravi Joshi
* Date: 2016/06/09
'''

import os
import numpy as np
from textwrap import wrap
import matplotlib.pyplot as plt

def main():
    print 'Plot performance\n'

    '''
    * Following values are recored manually running the compare_ik by changing input
    '''
    noise     = np.array([  0,   1,   2,   5,  10,  15,  20,  25,  30,  35, 40])
    kdl_perf  = np.array([ 99, 100,  99,  98,  93,  83,  63,  66,  65,  56, 55])
    trac_perf = np.array([100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99])

    # plot
    plt.figure(1)
    title = '\n'.join(wrap('Comparision of performance of inverse kinematics analysis by KDL and trac_ik', 60))
    plt.title(title)
    plt.xlabel('Gaussian Noise ('r'$\mu=0,\sigma$'') Unit degree')
    plt.ylabel('Perecentage of IK solutions found')
    plt.plot(noise,  kdl_perf, label='KDL')
    plt.plot(noise, trac_perf, label='trac_ik')
    plt.yticks(np.arange(0, 110, 10))
    plt.legend(loc='lower left')
    plt.ylim([0,110])

    filename = 'performance.png'
    print 'saving ' + filename + '\n'
    plt.savefig(filename)

if __name__ == "__main__":
        main()
