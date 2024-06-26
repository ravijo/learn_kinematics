/**
* util.h: Header file for utility functions
* Author: Ravi Joshi
* Date: 2016/06/08
*/

#ifndef FAST_KINEMATICS_UTIL_H_
#define FAST_KINEMATICS_UTIL_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/Float64MultiArray.h>

namespace kinematics {
  class util {
  public:

    /*
    * This function calulcates minimum and maximum limit of each joint in the provided kdl chain
    * Input:
    *   urdf::Model& robot_model = robot model
    *   KDL::Chain& chain = kdl chain of the robot model
    * Output:
    *   KDL::JntArray lower_limit = lower limit of all the joints
    *   KDL::JntArray upper_limit = upper limit of all the joints
    */
    static void get_joint_limits(const urdf::Model& robot_model, const KDL::Chain& chain, KDL::JntArray& lower_limit, KDL::JntArray& upper_limit);

    /*
    * This function calulcates kdl chain from urdf xml file
    * Input:
    *   std::string& urdf_xml = the content of urdf xml file
    *   std::string& base_link = base (start) link of the kdl chain
    *   std::string& tip_link = tip (end) link of the kdl chain
    * Output:
    *   KDL::Chain& chain = kdl chain
    * Returns
    *   urdf::Model = urdf model of the robot
    */
    static urdf::Model get_kdl_chain_from_urdf(const std::string& urdf_xml, const std::string& base_link, const std::string& tip_link, KDL::Chain& chain);

    /*
    * This function reads csv file, which contains floating point numbers
    * Input:
    *   std::string& file_name = absolute file name (full path) of the csv file
    *   int skip_rows = 1 = number of rows to skip while reading file from the top. Default is 1
    *   char delimiter  = ',' = delimiter  separating the cells in file. Default is comma
    * Output:
    *   std::vector<std::vector<double> >& data = 2D vector containing csv data
    */
    static void read_csv(const std::string& file_name, std::vector<std::vector<double> >& data, int skip_rows = 1, const char delimiter  = ',');

    /*
    * This function reads csv file, which contains integer numbers
    * Input:
    *   std::string& file_name = absolute file name (full path) of the csv file
    *   int skip_rows = 1 = number of rows to skip while reading file from the top. Default is 1
    *   char delimiter  = ',' = delimiter  separating the cells in file. Default is comma
    * Output:
    *   std::vector<std::vector<int> >& data = 2D vector containing csv data
    */
    static void read_csv(const std::string& file_name, std::vector<std::vector<int> >& data, int skip_rows = 1, const char delimiter  = ',');

    /*
    * This function write file. Note that, it creates a new file, if it doesn't exists..
    * otherwise it appends.
    * Input:
    *   std::string& file_name = absolute file name (full path)
    *   std::string content = content to write/append
    */
    static void write_file(const std::string& file_name, const std::string content);

    /*
    * This function creates string from joint array
    * Input:
    *   KDL::JntArray& joint_conf = joint configuration
    * Returns:
    *   std::string = converted string from input joint array
    */
    static std::string get_jntarray_as_string(const KDL::JntArray& joint_conf);

    /*
    * This function creates string from frame
    * Input:
    *   KDL::Frame& frame = frame
    * Returns:
    *   std::string = converted string from input frame
    */
    static std::string get_frame_as_string(const KDL::Frame& frame);

    /*
    * This function convert radian into degree
    * Input:
    *   double radian = angle in radian
    * Returns:
    *   double = converted angle in degree
    */
    static double degree(double radian);

    /*
    * This function convert degree into radian
    * Input:
    *   double degree = angle in degree
    * Returns:
    *   double = converted angle in radian
    */
    static double radian(double degree);

    /*
    * This function gets frame from standard floating point array
    * Input:
    *   int index = start index of the joint
    *   double frame_arr[] = joint configuration represented by array
    * Output:
    *   KDL::Frame& frame = frame converted from input array
    */
    static void get_frame_from_array(int index, double frame_arr[], KDL::Frame& frame);

    /*
    * This function sets jntarray from standard floating point array
    * Input:
    *   int index = start index of the joint
    *   int max_joint = maximum number of joints
    *   double frame_arr[] = joint configuration represented by array
    * Output:
    *   KDL::JntArray& joint_conf = joint configuration
    */
    static void get_jntarray_from_array(int index, int max_joint, double frame_arr[], KDL::JntArray& joint_conf);

    /*
    * This function sets jntarray from vector
    * Input:
    *   int index = start index of the joint
    *   int max_joint = maximum number of joints
    *   std::vector<double> &frame_arr = joint configuration represented by array
    * Output:
    *   KDL::JntArray& joint_conf = joint configuration
    */
    static void get_jntarray_from_vector(int index, int max_joint, const std::vector<double> &frame_arr, KDL::JntArray& joint_conf);
    /*
    * This function gets frame from vector
    * Input:
    *   int index = start index of the joint
    *   std::vector<double>& frame_arr = joint configuration represented by vector
    * Output:
    *   KDL::Frame& frame = frame converted from input array
    */
    static void get_frame_from_vector(int index, const std::vector<double>& frame_arr, KDL::Frame& frame);

    /*
    * This function sets array from jntarray
    * Input:
    *   KDL::JntArray& joint_conf = joint configuration
    *   int index = start index of the joint
    *   int max_joint = maximum number of joints
    * Output:
    *   std::vector<double>& joint_angles = array of joint angles
    */
    static void set_array_from_jntarray(int index, int max_joint, const KDL::JntArray& joint_conf, std::vector<double>& joint_angles);

    /*
    * This function sets standard ros floating point array from jntarray
    * Input:
    *   KDL::JntArray& joint_conf = joint configuration
    *   int index = start index of the joint
    *   int max_joint = maximum number of joints
    * Output:
    *   std_msgs::Float64MultiArray& arr = 1D array of joint angles
    */
    static void set_rosarray_from_jntarray(int index, int max_joint, const KDL::JntArray& joint_conf, std_msgs::Float64MultiArray& arr);

  private:

    /*
    * This template reads csv file
    * Input:
    *   std::string& file_name = absolute file name (full path) of the csv file
    *   int skip_rows = 1 = number of rows to skip while reading file from the top. Default is 1
    *   char delimiter  = ',' = delimiter  separating the cells in file. Default is comma
    * Output:
    *   std::vector<std::vector<T> >& data = 2D vector containing csv data
    */
    template <typename T> static void read_csv(const std::string& file_name, std::vector<std::vector<T> >& data, int skip_rows = 1, const char delimiter = ',');
  };
}

#endif				/* FAST_KINEMATICS_UTIL_H_ */
