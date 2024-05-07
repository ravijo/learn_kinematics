#!/usr/bin/env python

'''
* fast_kinematics_srv_test.py: It is a client for fast_kinematics_srv
* Author: Ravi Joshi
* Date: 2017/01/22
'''

import sys
import rospy
from learn_kinematics.srv import fast_kinematics_service

def fast_kinematics_service_client():
	print "Waiting for service fast_kinematics_service"
	rospy.wait_for_service('fast_kinematics_service')

	print "Waiting for service fast_kinematics_service_left"
	rospy.wait_for_service('fast_kinematics_service_left')

	print "Waiting for service fast_kinematics_service_right"
	rospy.wait_for_service('fast_kinematics_service_right')
	try:
		print "Calling service"
		service_request_left  = rospy.ServiceProxy('fast_kinematics_service_left', fast_kinematics_service)
		service_request_right = rospy.ServiceProxy('fast_kinematics_service_right', fast_kinematics_service)
		service_request_both  = rospy.ServiceProxy('fast_kinematics_service', fast_kinematics_service)

		'''
		* pose is an array consisting of 14 elements.
		* First 7 elements represents the end-effector pose of left arm
		* Later 7 elements represents the end-effector pose of right arm
		* Note that the pose contains position cartesian system and orientaion in quaternion system
		* Example:
		* pose = [poselinear_left_x,poselinear_left_y,poselinear_left_z,poseangle_left_x,poseangle_left_y,poseangle_left_z,poseangle_left_w,
		*  poselinear_right_x,poselinear_right_y,poselinear_right_z,poseangle_right_x,poseangle_right_y,poseangle_right_z,poseangle_right_w]
		'''
		pose_left  = [0.7966,  0.1354, 0.3337, -0.3709, 0.6865, -0.5933, 0.1979]
		pose_right = [0.7979, -0.1427, 0.3390,  0.3577, 0.7037,  0.5807, 0.1989]
		pose_both  = pose_left + pose_right

		'''
		* joint_seed is an array consisting of 14 elements.
		* First 7 elements represents the joint angle of left arm
		* Later 7 elements represents the joint angle of right arm
		* Example:
		* joint_seed = [left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,
		*        right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2]
		'''
		joint_seed_left  = [ 0.2282, -0.2531, -1.4105, 1.8573, -0.1277, -0.1292, -0.3279]
		joint_seed_right = [-0.2286, -0.2504,  1.4155, 1.8553,  0.1170, -0.1204,  0.3279]
		joint_seed_both   = joint_seed_left + joint_seed_right

		service_response_left  = service_request_left(pose_left,  joint_seed_left)
		service_response_right = service_request_right(pose_right, joint_seed_right)
		service_response_both  = service_request_both(pose_both,  joint_seed_both)

		# print the joint angles calculated by inverse_kinematics
		print 'Left Arm:' , service_response_left.success_left, service_response_left.joint_angles
		print 'Right Arm:', service_response_right.success_right, service_response_right.joint_angles
		print 'Both Arm:' , service_response_both.success_left, service_response_both.success_right, service_response_both.joint_angles
		return
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	fast_kinematics_service_client()
