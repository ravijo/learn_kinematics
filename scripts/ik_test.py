#!/usr/bin/env python

'''
Summary: In order to work with this IK solver, make sure to follow steps below:
(1) Open terminal and source baxter.sh file
(2) Run following command: rosrun baxter_examples send_urdf_fragment.py -f `rospack find baxter_description`/urdf/left_end_effector.urdf.xacro -l left_hand -j left_gripper_base
(3) Run following command: rosrun baxter_examples send_urdf_fragment.py -f `rospack find baxter_description`/urdf/right_end_effector.urdf.xacro -l right_hand -j right_gripper_base
(4) Add 0.01 to the z coordinate of the end-effector pose
'''

import sys
import rospy
import numpy as np
import baxter_interface
from learn_kinematics.srv import fast_kinematics_service

def fast_kinematics_service_client():
	rospy.init_node("IK_test")
	limb = 'right'
	arm = baxter_interface.Limb(limb)

	arm_pose = arm.endpoint_pose()
	arm_ja = arm.joint_angles()

	print "Waiting for service fast_kinematics_service_right"
	rospy.wait_for_service('fast_kinematics_service_'+ limb)

	try:
		print "Calling service"
		service_request_left  = rospy.ServiceProxy('fast_kinematics_service_'+ limb, fast_kinematics_service)

		arm_pose_list  = [arm_pose['position'].x,  +\
						  arm_pose['position'].y,  +\
						  arm_pose['position'].z + 0.01] +\
					 list(arm_pose['orientation'])

		joint_seed_left  = [ arm_ja[limb + '_s0'], arm_ja[limb + '_s1'], \
							 arm_ja[limb + '_e0'], arm_ja[limb + '_e1'], \
							 arm_ja[limb + '_w0'], arm_ja[limb + '_w1'], \
							 arm_ja[limb + '_w2']]

		service_response_left  = service_request_left(arm_pose_list,  joint_seed_left)

		left_new_ja = list(service_response_left.joint_angles)

		command = {limb + '_s0': left_new_ja[0], limb + '_s1': left_new_ja[1],\
				   limb + '_e0': left_new_ja[2], limb + '_e1': left_new_ja[3],\
				   limb + '_w0': left_new_ja[4], limb + '_w1': left_new_ja[5],\
				   limb + '_w2': left_new_ja[6]}
		arm.move_to_joint_positions(command)

		old_pose = np.array(arm_pose['position'])
		new_pose = np.array(arm.endpoint_pose()['position'])

		diff = (new_pose - old_pose).tolist()

		print 'Before EE:', arm_pose_list[:3]
		print 'After  EE:', list(arm.endpoint_pose()['position'])
		print 'diff   EE:', diff

		return
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	fast_kinematics_service_client()
