#!/usr/bin/env python

import argparse
import rospy
import baxter_interface
import recorder
import moveit_commander
from baxter_interface import CHECK_VERSION


def main():
    """Joint and EE Pose Recorder
    Record timestamped joint and gripper positions to a file
    """
    epilog = """
Related examples:
  joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    parser.add_argument('-f', dest='filename', type = str, required=True, help = 'Output Joint Angle Filename')
    parser.add_argument('-r', dest='record_rate', type=int, default=100, metavar='RECORDRATE', help='rate at which to record (default: 100)')
    parser.add_argument('-m', dest='mode', type = int, default=1, help = 'JointRecorder mode: 0=Only JA, 1=All data (default: 1)')
    
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_recorder")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    #recordingRate = 100
    #mode = 1
    baxter_recorder = recorder.JointRecorder(args.filename, args.record_rate, args.mode)
    rospy.on_shutdown(baxter_recorder.stop)

    print("Recording. Press Ctrl-C to stop.")
    baxter_recorder.record()

    print("\nDone.")

if __name__ == '__main__':
    main()
