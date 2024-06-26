/**
* fast_kinematics_test.cpp: Test file for fast_kinematics.cpp
* Author: Ravi Joshi
* Date: 2016/06/09
*/

#include <util.h>
#include <fast_kinematics.h>

int main(int argc, char** argv)
{
  if ( argc < 2 )
  {
    ROS_FATAL("Don't run this file directly, instead use roslaunch\n\tcommand: roslaunch learn_kinematics fast_kinematics_test.launch\n");
    exit(-1);
  }

  ros::init(argc, argv, "fast_kinematics_test_node");

  /*
  * Define global namespace
  * src: http://dougsbots.blogspot.jp/2012/09/ros-launch-files-and-parameters.html
  */
  ros::NodeHandle nh;

  // read the following parameter provided in launch file
  double timeout;
  int num_joints, left_joint_offset, right_joint_offset;
  std::string left_chain_start, right_chain_start, left_chain_end, right_chain_end, urdf_xml, data_file, ikin_output_file, joint_array_header, ee_pose_header;

  nh.param("/kinematics/left_chain_start", left_chain_start, std::string(""));
  nh.param("/kinematics/right_chain_start", right_chain_start, std::string(""));
  nh.param("/kinematics/left_chain_end", left_chain_end, std::string(""));
  nh.param("/kinematics/right_chain_end", right_chain_end, std::string(""));
  nh.param("/kinematics/urdf_xml", urdf_xml, std::string(""));

  nh.param("/kinematics/timeout", timeout, 0.005);
  nh.param("/kinematics/num_joints", num_joints, 7);
  nh.param("/kinematics/left_joint_offset", left_joint_offset, 0);
  nh.param("/kinematics/right_joint_offset", right_joint_offset, 7);

  nh.param("/kinematics/input_file", data_file, std::string(""));
  nh.param("/kinematics/output_file", ikin_output_file, std::string(""));
  nh.param("/kinematics/joint_array_header", joint_array_header, std::string(""));
  nh.param("/kinematics/ee_pose_header", ee_pose_header, std::string(""));

  if (left_chain_start == "" || right_chain_start == "" || left_chain_end == "" || right_chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  ROS_INFO_STREAM("Input File: " << data_file);
  ROS_INFO_STREAM("Output File: " << ikin_output_file);

  // write the headers to the csv files
  kinematics::util::write_file(ikin_output_file, joint_array_header);

  KDL::Chain left_chain, right_chain;
  urdf::Model left_model = kinematics::util::get_kdl_chain_from_urdf(urdf_xml, left_chain_start, left_chain_end, left_chain);
  urdf::Model right_model = kinematics::util::get_kdl_chain_from_urdf(urdf_xml, right_chain_start, right_chain_end, right_chain);

  KDL::JntArray left_lower_limit, left_upper_limit, right_lower_limit, right_upper_limit;
  kinematics::util::get_joint_limits(left_model, left_chain, left_lower_limit, left_upper_limit);
  kinematics::util::get_joint_limits(right_model, right_chain, right_lower_limit, right_upper_limit);

  kinematics::fast_kinematics left_kin(left_chain, left_lower_limit, left_upper_limit, timeout);
  kinematics::fast_kinematics right_kin(right_chain, right_lower_limit, right_upper_limit, timeout);

  std::vector<std::vector<double> > csv_data;
  kinematics::util::read_csv(data_file, csv_data);

  std::vector<KDL::JntArray> left_joint_list, right_joint_list;
  int num_samples = csv_data.size();
  int colums = csv_data[0].size();
  for (int i = 0; i < num_samples; i++)
  {
    KDL::JntArray left(num_joints), right(num_joints);
    int left_count = 0;
    int right_count = 0;

    for (int j = 0; j < colums; j++)
    {
      if (j >= left_joint_offset && j < left_joint_offset + num_joints)
      {
        left(left_count++) = csv_data.at(i).at(j);
      }
      else if (j >= right_joint_offset && j < right_joint_offset + num_joints)
      {
        right(right_count++) = csv_data.at(i).at(j);
      }
    }
    left_joint_list.push_back(left);
    right_joint_list.push_back(right);
  }

  uint left_fkin_success = 0;
  uint right_fkin_success = 0;
  uint left_ikin_success = 0;
  uint right_ikin_success = 0;

  // initialize with first joint configuration
  KDL::JntArray left_joint_seed = left_joint_list[0];
  KDL::JntArray right_joint_seed = right_joint_list[0];
  for (uint i = 0; i < num_samples; i++)
  {
    KDL::JntArray left_joint_conf = left_joint_list[i];
    KDL::JntArray right_joint_conf = right_joint_list[i];
    KDL::Frame left_ee_pose, right_ee_pose;

    int fkin_left_out = left_kin.forward_kinematics(left_joint_conf, left_ee_pose);
    int fkin_right_out = right_kin.forward_kinematics(right_joint_conf, right_ee_pose);

    if (fkin_left_out >= 0)
    {
      left_fkin_success++;
    }

    if (fkin_right_out >= 0)
    {
      right_fkin_success++;
    }

    int ikin_left_out = left_kin.inverse_kinematics(left_ee_pose, left_joint_conf, left_joint_seed);
    int ikin_right_out = right_kin.inverse_kinematics(right_ee_pose, right_joint_conf, right_joint_seed);

    // store the current output for next seed
    left_joint_seed = left_joint_conf;
    right_joint_seed = right_joint_conf;

    if (ikin_left_out >= 0)
    {
      left_ikin_success++;
    }

    if (ikin_right_out >= 0)
    {
      right_ikin_success++;
    }

    // append the result of inverse kinematics to csv
    kinematics::util::write_file(ikin_output_file, kinematics::util::get_jntarray_as_string(left_joint_conf) + "," +
    kinematics::util::get_jntarray_as_string(right_joint_conf));
  }

  ROS_INFO_STREAM("\nTotal samples = " << num_samples << "\t left arm \t right arm \n\tforward kinematics success \t"
  << left_fkin_success << "\t" << right_fkin_success << "\n\tinverse kinematics success \t"
  << left_ikin_success << "\t" << right_ikin_success);
  return 0;
} 
