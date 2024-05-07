/**
* fast_kinematics_pub.cpp: publisher designed to use fast_kinematics.cpp
* Author: Ravi Joshi
* Date: 2016/06/08
*/

//utility
#include <util.h>
#include <fast_kinematics.h>
#include <std_msgs/Float64MultiArray.h>

// Baxter have two arms, so we are allocating end-effector frame for both the arms
double *both_ee_pose;

// Callback function definition. It will be called whenever end-effector pose is received
void pose_listener_callback(const std_msgs::Float64MultiArray::ConstPtr& pose_array);

int main(int argc, char** argv)
{
	if ( argc < 2 )
	{
		ROS_FATAL("Don't run this file directly, instead use roslaunch\n\tcommand: roslaunch learn_kinematics fast_kinematics_pub.launch\n");
		exit(-1);
	}

	ros::init(argc, argv, "fast_kinematics_node");
	ros::NodeHandle nh;

	// read the following parameter provided in launch file
	double timeout;
	int queue_size, num_joints, left_joint_offset, right_joint_offset;
	std::string left_chain_start, right_chain_start, left_chain_end, right_chain_end, urdf_xml;

	nh.param("/kinematics/left_chain_start", left_chain_start, std::string(""));
	nh.param("/kinematics/right_chain_start", right_chain_start, std::string(""));
	nh.param("/kinematics/left_chain_end", left_chain_end, std::string(""));
	nh.param("/kinematics/right_chain_end", right_chain_end, std::string(""));
	nh.param("/kinematics/urdf_xml", urdf_xml,  std::string(""));

	nh.param("/kinematics/timeout", timeout, 0.005);
	nh.param("/kinematics/num_joints", num_joints, 7);
	nh.param("/kinematics/left_joint_offset", left_joint_offset, 0);
	nh.param("/kinematics/right_joint_offset", right_joint_offset, 7);

	if (left_chain_start == "" || right_chain_start == "" || left_chain_end == "" || right_chain_end == "")
	{
		ROS_FATAL("Missing chain info in launch file");
		exit(-1);
	}

	/*
	* Baxter have two arms, so we are allocating end-effector frame
	* and joint configuration for both the arms
	*/
	both_ee_pose = new double[2 * num_joints];
	std_msgs::Float64MultiArray both_joint_conf;

	//make a subscriber to receive the end-effector pose from user
	ros::Subscriber subscriber = nh.subscribe("fast_kinematics_listener", queue_size, pose_listener_callback);

	//make a publisher to publish the soultion of inverse kinematics using received end-effector pose
	ros::Publisher publisher = nh.advertise<std_msgs::Float64MultiArray>("fast_kinematics_publisher", queue_size);

	KDL::Chain left_chain, right_chain;
	urdf::Model left_model = kinematics::util::get_kdl_chain_from_urdf(urdf_xml, left_chain_start, left_chain_end, left_chain);
	urdf::Model right_model = kinematics::util::get_kdl_chain_from_urdf(urdf_xml, right_chain_start, right_chain_end, right_chain);

	KDL::JntArray left_lower_limit, left_upper_limit, right_lower_limit, right_upper_limit;
	kinematics::util::get_joint_limits(left_model, left_chain, left_lower_limit, left_upper_limit);
	kinematics::util::get_joint_limits(right_model, right_chain, right_lower_limit, right_upper_limit);

	kinematics::fast_kinematics left_kin(left_chain, left_lower_limit, left_upper_limit, timeout);
	kinematics::fast_kinematics right_kin(right_chain, right_lower_limit, right_upper_limit, timeout);

	// initialize with nominal joint configuration
	KDL::JntArray  left_joint_seed(num_joints);
	KDL::JntArray right_joint_seed(num_joints);
	left_kin.get_nominal_joint_conf(left_joint_seed);
	right_kin.get_nominal_joint_conf(right_joint_seed);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		both_joint_conf.data.clear();
		both_joint_conf.data.resize(2 * num_joints);

		// initialize frame for performing inverse kinematics
		KDL::Frame left_ee_pose, right_ee_pose;

		// fill the frame from received container
		kinematics::util::get_frame_from_array(left_joint_offset, both_ee_pose, left_ee_pose);
		kinematics::util::get_frame_from_array(right_joint_offset, both_ee_pose, right_ee_pose);

		// initialize frame to store the output of inverse kinematics
		KDL::JntArray left_joint_conf, right_joint_conf;

		/* Just for debugging purpose
		ROS_INFO_STREAM("Left ee pose is " << kinematics::util::get_frame_as_string(left_ee_pose));
		ROS_INFO_STREAM("Left ee seed is " << kinematics::util::get_jntarray_as_string(left_joint_seed));
		ROS_INFO_STREAM("Right ee pose is " << kinematics::util::get_frame_as_string(right_ee_pose));
		ROS_INFO_STREAM("Right ee seed is " << kinematics::util::get_jntarray_as_string(right_joint_seed));
		*/

		//call the inverse kinematics module for each arm
		int ikin_left  = left_kin.inverse_kinematics(left_ee_pose, left_joint_conf, left_joint_seed);
		int ikin_right = right_kin.inverse_kinematics(right_ee_pose, right_joint_conf, right_joint_seed);

		//If inverse kinematics is succeeded, update the joint seed and set the joint configuration
		if (ikin_left >= 0)
		{
			left_joint_seed = left_joint_conf;
			kinematics::util::set_rosarray_from_jntarray(left_joint_offset, num_joints, left_joint_conf, both_joint_conf);
		}
		else
		{
			ROS_WARN_STREAM("Error in inverse kinematics for left arm. End-Effector pose is {" <<
				 kinematics::util::get_frame_as_string(left_ee_pose) <<
				 "}. Are you publishing the data to fast_kinematics_listener topic?");
		}

		if (ikin_right >= 0)
		{
			right_joint_seed = right_joint_conf;
			kinematics::util::set_rosarray_from_jntarray(right_joint_offset, num_joints, right_joint_conf, both_joint_conf);
		}
		else
		{
			ROS_WARN_STREAM("Error in inverse kinematics for right arm. End-Effector pose is {" <<
				 kinematics::util::get_frame_as_string(right_ee_pose) <<
				 "}. Are you publishing the data to fast_kinematics_listener topic?");
		}

		//publish the joint configuration for both the joints
		publisher.publish(both_joint_conf);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}

void pose_listener_callback(const std_msgs::Float64MultiArray::ConstPtr& pose_array)
{
	//just store the received end-effector pose in a container
	uint i = 0;
	for(std::vector<double>::const_iterator it = pose_array->data.begin(); it != pose_array->data.end(); ++it)
	{
		both_ee_pose[i++] = *it;
	}
}
