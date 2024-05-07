/**
* fast_kinematics_pub_test.cpp: test file for fast_kinematics_pub.cpp
* Author: Ravi Joshi
* Date: 2016/06/08
*/

#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_only_node_test");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("fast_kinematics_listener", 10);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    array.data.clear();

    array.data.push_back(0.8047); //poselinear_left_x
    array.data.push_back(0.1436); //poselinear_left_y
    array.data.push_back(0.3308); //poselinear_left_z
    array.data.push_back(-0.4132);//poseangle_left_x
    array.data.push_back(0.6569); //poseangle_left_y
    array.data.push_back(-0.5960);//poseangle_left_z
    array.data.push_back(0.2062); //poseangle_left_w
    array.data.push_back(0.8034); //poselinear_right_x
    array.data.push_back(-0.1467);//poselinear_right_y
    array.data.push_back(0.3328); //poselinear_right_z
    array.data.push_back(0.4093); //poseangle_right_x
    array.data.push_back(0.6593); //poseangle_right_y
    array.data.push_back(0.5946); //poseangle_right_z
    array.data.push_back(0.2100); //poseangle_right_w

    //Publish array
    pub.publish(array);

    std::string value = "";
    for(std::vector<double>::const_iterator it = array.data.begin(); it != array.data.end(); ++it)
    {
      double d = *it;
      value = value + boost::lexical_cast<std::string>(d) + ",";
    }

    ROS_INFO_STREAM("Published {" << value << "}\n");

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
