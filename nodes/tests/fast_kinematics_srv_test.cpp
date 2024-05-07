/**
* fast_kinematics_srv_test.cpp: Test file for fast_kinematics_srv.cpp
* It is a client for fast_kinematics_srv
* Author: Ravi Joshi
* Date: 2016/06/09
*/

#include <ros/ros.h>
#include <learn_kinematics/fast_kinematics_service.h>

const int NUM_JOINTS = 7; // total number of joints in baxter arm

std::string vector_to_string(std::vector<double>& v);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_kinematics_service_node_test");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<learn_kinematics::fast_kinematics_service>("fast_kinematics_service");

 /*
  * joint_seed is an array consisting of 14 elements.
  * First 7 elements represents the joint angle of left arm
  * Later 7 elements represents the joint angle of right arm
  * Example:
  * joint_seed = [left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,
  *        right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2]
  */
  std::vector<double> joint_seed;
  joint_seed.push_back(0.2282); //left_s0
  joint_seed.push_back(-0.2531);//left_s1
  joint_seed.push_back(-1.4105);//left_e0
  joint_seed.push_back(1.8573); //left_e1
  joint_seed.push_back(-0.1277);//left_w0
  joint_seed.push_back(-0.1292);//left_w1
  joint_seed.push_back(-0.3279);//left_w2
  joint_seed.push_back(-0.2286);//right_s0
  joint_seed.push_back(-0.2504);//right_s1
  joint_seed.push_back(1.4155); //right_e0
  joint_seed.push_back(1.8553); //right_e1
  joint_seed.push_back(0.1170); //right_w0
  joint_seed.push_back(-0.1204);//right_w1
  joint_seed.push_back(0.3279); //right_w2

  /*
  * pose is an array consisting of 14 elements.
  * First 7 elements represents the end-effector pose of left arm
  * Later 7 elements represents the end-effector pose of right arm
  * Note that the pose contains position cartesian system and orientaion in quaternion system
  * Example:
  * pose = [poselinear_left_x,poselinear_left_y,poselinear_left_z,poseangle_left_x,poseangle_left_y,poseangle_left_z,poseangle_left_w,
  *  poselinear_right_x,poselinear_right_y,poselinear_right_z,poseangle_right_x,poseangle_right_y,poseangle_right_z,poseangle_right_w]
  */
  std::vector<double> ee_pose;
  ee_pose.push_back(0.7966); //poselinear_left_x
  ee_pose.push_back(0.1354); //poselinear_left_y
  ee_pose.push_back(0.3337); //poselinear_left_z
  ee_pose.push_back(-0.3709);//poseangle_left_x
  ee_pose.push_back(0.6865); //poseangle_left_y
  ee_pose.push_back(-0.5933);//poseangle_left_z
  ee_pose.push_back(0.1979); //poseangle_left_w
  ee_pose.push_back(0.7979); //poselinear_right_x
  ee_pose.push_back(-0.1427);//poselinear_right_y
  ee_pose.push_back(0.3390); //poselinear_right_z
  ee_pose.push_back(0.3577); //poseangle_right_x
  ee_pose.push_back(0.7037); //poseangle_right_y
  ee_pose.push_back(0.5807); //poseangle_right_z
  ee_pose.push_back(0.1989); //poseangle_right_w

  learn_kinematics::fast_kinematics_service service;
  service.request.pose = ee_pose;
  service.request.joint_seed = joint_seed;

  if (client.call(service))
  {
    ROS_INFO_STREAM("Clinet Response " << vector_to_string(service.response.joint_angles));
  }
  else
  {
    ROS_ERROR("Failed to call service fast_kinematics_service");
    return -1;
  }

  return 0;
}

std::string vector_to_string(std::vector<double>& v)
{
  std::string val = "{" ;
  for (int i = 0; i < (v.size() - 1); i++)
  {
    val = val + boost::lexical_cast<std::string>(v[i]) + ",";
  }
  val = val + boost::lexical_cast<std::string>(v[v.size() - 1]) + "}";

  return val;
}
