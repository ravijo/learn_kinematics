/**
* fast_kinematics_srv.cpp: service designed to use fast_kinematics.cpp
* Author: Ravi Joshi
* Date: 2016/06/08
*/

#include <util.h>
#include <fast_kinematics.h>
#include <learn_kinematics/fast_kinematics_service.h>

// Declare left and right arm kinematics solver
kinematics::fast_kinematics* left_kin;
kinematics::fast_kinematics* right_kin;

// following variables are provide inside launch file
int num_joints, left_joint_offset, right_joint_offset;

// Callback function declaration. it will be called whenever client send service request
bool pose_service_callback(learn_kinematics::fast_kinematics_service::Request &req, learn_kinematics::fast_kinematics_service::Response &res);
bool pose_service_callback_left(learn_kinematics::fast_kinematics_service::Request &req, learn_kinematics::fast_kinematics_service::Response &res);
bool pose_service_callback_right(learn_kinematics::fast_kinematics_service::Request &req, learn_kinematics::fast_kinematics_service::Response &res);

int main(int argc, char **argv)
{
	if ( argc < 2 )
	{
		ROS_FATAL("Don't run this file directly, instead use roslaunch\n\tcommand: roslaunch learn_kinematics fast_kinematics_srv.launch\n");
		exit(-1);
	}

	ros::init(argc, argv, "fast_kinematics_service_node");

	/*
	* Define global namespace
	* src: http://dougsbots.blogspot.jp/2012/09/ros-launch-files-and-parameters.html
	*/
	ros::NodeHandle nh;

	// read the following parameter provided in launch file
	double timeout;
	std::string left_chain_start, right_chain_start, left_chain_end, right_chain_end, urdf_xml;

	nh.param("/kinematics/left_chain_start", left_chain_start, std::string(""));
	nh.param("/kinematics/right_chain_start", right_chain_start, std::string(""));
	nh.param("/kinematics/left_chain_end", left_chain_end, std::string(""));
	nh.param("/kinematics/right_chain_end", right_chain_end, std::string(""));
	nh.param("/kinematics/urdf_xml", urdf_xml, std::string(""));

	nh.param("/kinematics/timeout", timeout, 0.005);
	nh.param("/kinematics/num_joints", num_joints, 7);
	nh.param("/kinematics/left_joint_offset", left_joint_offset, 0);
	nh.param("/kinematics/right_joint_offset", right_joint_offset, 7);

	if (left_chain_start == "" || right_chain_start == "" || left_chain_end == "" || right_chain_end == "")
	{
		ROS_FATAL("Missing chain info in launch file");
		exit(-1);
	}

	ros::ServiceServer service   = nh.advertiseService("fast_kinematics_service", pose_service_callback);
	ros::ServiceServer service_l = nh.advertiseService("fast_kinematics_service_left", pose_service_callback_left);
	ros::ServiceServer service_r = nh.advertiseService("fast_kinematics_service_right", pose_service_callback_right);

	KDL::Chain left_chain, right_chain;
	urdf::Model left_model = kinematics::util::get_kdl_chain_from_urdf(urdf_xml, left_chain_start, left_chain_end, left_chain);
	urdf::Model right_model = kinematics::util::get_kdl_chain_from_urdf(urdf_xml, right_chain_start, right_chain_end, right_chain);

	KDL::JntArray left_lower_limit, left_upper_limit, right_lower_limit, right_upper_limit;
	kinematics::util::get_joint_limits(left_model, left_chain, left_lower_limit, left_upper_limit);
	kinematics::util::get_joint_limits(right_model, right_chain, right_lower_limit, right_upper_limit);

	left_kin  = new kinematics::fast_kinematics(left_chain, left_lower_limit, left_upper_limit, timeout);
	right_kin = new kinematics::fast_kinematics(right_chain, right_lower_limit, right_upper_limit, timeout);

	ros::spin();

	return 0;
}

bool pose_service_callback_left(learn_kinematics::fast_kinematics_service::Request &req, learn_kinematics::fast_kinematics_service::Response &res)
{
	KDL::JntArray left_joint_conf;
	KDL::Frame left_ee_pose;
	kinematics::util::get_frame_from_vector(left_joint_offset, req.pose, left_ee_pose);

	KDL::JntArray left_joint_seed(num_joints);
	kinematics::util::get_jntarray_from_vector(left_joint_offset, num_joints, req.joint_seed, left_joint_seed);

	/* For debugging purpose*/
	ROS_DEBUG_STREAM("Left ee pose is " << kinematics::util::get_frame_as_string(left_ee_pose));
	ROS_DEBUG_STREAM("Left ee seed is " << kinematics::util::get_jntarray_as_string(left_joint_seed));
	/* */

	int ikin_left  = left_kin->inverse_kinematics(left_ee_pose, left_joint_conf, left_joint_seed);

	//considering only one arm
	res.joint_angles.resize(num_joints);
	res.success_left  = ikin_left;

	if (ikin_left >= 0)
	{
		kinematics::util::set_array_from_jntarray(left_joint_offset, num_joints, left_joint_conf, res.joint_angles);
	}
	else
	{
		ROS_WARN_STREAM("Error in inverse kinematics for left arm. End-Effector pose is " << kinematics::util::get_frame_as_string(left_ee_pose));
	}

	return true;
}

bool pose_service_callback_right(learn_kinematics::fast_kinematics_service::Request &req, learn_kinematics::fast_kinematics_service::Response &res)
{
	KDL::JntArray right_joint_conf;
	KDL::Frame right_ee_pose;
	kinematics::util::get_frame_from_vector(left_joint_offset, req.pose, right_ee_pose);

	KDL::JntArray right_joint_seed(num_joints);
	kinematics::util::get_jntarray_from_vector(left_joint_offset, num_joints, req.joint_seed, right_joint_seed);

	/* For debugging purpose*/
	ROS_DEBUG_STREAM("Right ee pose is " << kinematics::util::get_frame_as_string(right_ee_pose));
	ROS_DEBUG_STREAM("Right ee seed is " << kinematics::util::get_jntarray_as_string(right_joint_seed));
	/* */

	int ikin_right = right_kin->inverse_kinematics(right_ee_pose, right_joint_conf, right_joint_seed);

	//considering only one arm
	res.joint_angles.resize(num_joints);
	res.success_right = ikin_right;

	if (ikin_right >= 0)
	{
		kinematics::util::set_array_from_jntarray(left_joint_offset, num_joints, right_joint_conf, res.joint_angles);
	}
	else
	{
		ROS_WARN_STREAM("Error in inverse kinematics for right arm. End-Effector pose is " << kinematics::util::get_frame_as_string(right_ee_pose));
	}

	return true;
}

bool pose_service_callback(learn_kinematics::fast_kinematics_service::Request &req, learn_kinematics::fast_kinematics_service::Response &res)
{
	KDL::JntArray left_joint_conf, right_joint_conf;
	KDL::Frame left_ee_pose, right_ee_pose;
	kinematics::util::get_frame_from_vector(left_joint_offset, req.pose, left_ee_pose);
	kinematics::util::get_frame_from_vector(right_joint_offset, req.pose, right_ee_pose);

	KDL::JntArray left_joint_seed(num_joints);
	KDL::JntArray right_joint_seed(num_joints);
	kinematics::util::get_jntarray_from_vector(left_joint_offset, num_joints, req.joint_seed, left_joint_seed);
	kinematics::util::get_jntarray_from_vector(right_joint_offset, num_joints, req.joint_seed, right_joint_seed);

	/* For debugging purpose*/
	ROS_INFO_STREAM("Left ee pose is " << kinematics::util::get_frame_as_string(left_ee_pose));
	ROS_INFO_STREAM("Left ee seed is " << kinematics::util::get_jntarray_as_string(left_joint_seed));
	ROS_INFO_STREAM("Right ee pose is " << kinematics::util::get_frame_as_string(right_ee_pose));
	ROS_INFO_STREAM("Right ee seed is " << kinematics::util::get_jntarray_as_string(right_joint_seed));
	/* */

	int ikin_left  = left_kin->inverse_kinematics(left_ee_pose, left_joint_conf, left_joint_seed);
	int ikin_right = right_kin->inverse_kinematics(right_ee_pose, right_joint_conf, right_joint_seed);

	//Baxter have two arms
	res.joint_angles.resize(2 * num_joints);
	res.success_left  = ikin_left;
	res.success_right = ikin_right;

	if (ikin_left >= 0)
	{
		kinematics::util::set_array_from_jntarray(left_joint_offset, num_joints, left_joint_conf, res.joint_angles);
	}
	else
	{
		ROS_WARN_STREAM("Error in inverse kinematics for left arm. End-Effector pose is " << kinematics::util::get_frame_as_string(left_ee_pose));
	}

	if (ikin_right >= 0)
	{
		kinematics::util::set_array_from_jntarray(right_joint_offset, num_joints, right_joint_conf, res.joint_angles);
	}
	else
	{
		ROS_WARN_STREAM("Error in inverse kinematics for right arm. End-Effector pose is " << kinematics::util::get_frame_as_string(right_ee_pose));
	}

	return true;
}
