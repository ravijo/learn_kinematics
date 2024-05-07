/**
* compare_ik.cpp: File for comparing the inverse inematics calculated by orocos kdl and trac_ik
* Author: Ravi Joshi
* Date: 2016/06/09
*/

#include <util.h>
#include <chrono>
#include <boost/date_time.hpp>
#include <random>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

/*
* num_joints = total number of joints in baxter arm
* left_joint_offset = starting postion of left joint in csv file
*/
int num_joints, left_joint_offset;

// we need header in csv file to make them more readable
std::string joint_array_header, ee_pose_header;

void test(const KDL::Chain& chain, const KDL::JntArray& lower_limit, const KDL::JntArray& upper_limit, double timeout, std::vector<std::vector<double> >& csv_data, std::string out_location, double noise)
{
  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain, lower_limit, upper_limit, timeout, eps);

  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, lower_limit, upper_limit, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    //        nominal(j) = (ll(j) + ul(j)) / 2.0;
    nominal(j) = csv_data.at(0).at(j + left_joint_offset);
  }

  // Create desired number of valid, random joint configurations
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  int num_samples = csv_data.size();

  for(int i = 0; i < num_samples; i++)
  {
    for(int j = 0; j < num_joints; j++)
    {
      q(j) = csv_data.at(i).at(j + left_joint_offset);
    }
    JointList.push_back(q);
  }

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  int rc;

  double total_time = 0;
  uint success = 0;

  ROS_INFO_STREAM("*** Testing KDL with " << num_samples << " samples");

  // setup files for saving data for plotting porpose
  std::string fkin_input_file       = out_location + "/fkin_input.csv";
  std::string fkin_kdl_output_file  = out_location + "/fkin_kdl_output.csv";
  std::string ikin_kdl_output_file  = out_location + "/ikin_kdl_output_gauss_" + boost::lexical_cast<std::string>(noise) + ".csv";
  std::string fkin_trac_output_file = out_location + "/fkin_trck_output.csv";
  std::string ikin_trac_output_file = out_location + "/ikin_trck_output_gauss_" + boost::lexical_cast<std::string>(noise) + ".csv";

  // write the headers to the csv files
  kinematics::util::write_file(fkin_input_file, joint_array_header);
  kinematics::util::write_file(fkin_kdl_output_file, ee_pose_header);
  kinematics::util::write_file(ikin_kdl_output_file, joint_array_header);
  kinematics::util::write_file(fkin_trac_output_file, ee_pose_header);
  kinematics::util::write_file(ikin_trac_output_file, joint_array_header);

  KDL::Frame end_effector_pose;
  KDL::JntArray result;
  result = nominal; // start with nominal

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, kinematics::util::radian(noise));
  // Initialize nosiy seed configuration
  KDL::JntArray noisy_seed(chain.getNrOfJoints());

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose);

    // save the current joint configuration and corrosponding end-effector pose
    kinematics::util::write_file(fkin_input_file, kinematics::util::get_jntarray_as_string(JointList[i]));
    kinematics::util::write_file(fkin_kdl_output_file, kinematics::util::get_frame_as_string(end_effector_pose));

    double elapsed = 0;
    //        result = nominal; // start with nominal
    start_time = boost::posix_time::microsec_clock::local_time();

    for (uint j = 0; j < noisy_seed.data.size(); j++)
    {
      noisy_seed(j) = result(j) + distribution(generator);
    }

    result = noisy_seed;

    // main iteration
    do
    {
      q = result;
      //            q = result; // when iterating start with last solution
      rc = kdl_solver.CartToJnt(q, end_effector_pose, result);
      diff = boost::posix_time::microsec_clock::local_time() - start_time;
      elapsed = diff.total_nanoseconds() / 1e9;

    } while (rc < 0 && elapsed < timeout);
    total_time += elapsed;
    if (rc >= 0)
    {
      // save the result of inverse kinematics
      kinematics::util::write_file(ikin_kdl_output_file, kinematics::util::get_jntarray_as_string(result));
      success++;
    }
    else
    {
      // save the result of inverse kinematics
      kinematics::util::write_file(ikin_kdl_output_file, kinematics::util::get_jntarray_as_string(nominal));
    }

    if (int((double)i / num_samples * 100) % 10 == 0)
    ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }

  ROS_INFO_STREAM("KDL found " << success << " solutions (" << 100.0 * success / num_samples
  << "\%) with an average of " << total_time / num_samples
  << " secs per sample");


  total_time = 0;
  success = 0;

  ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " samples");

  result = nominal; // start with nominal

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose);

    // save the forward kinematics result i.e., end-effector pose
    kinematics::util::write_file(fkin_trac_output_file, kinematics::util::get_frame_as_string(end_effector_pose));

    for (uint j = 0; j < noisy_seed.data.size(); j++)
    {
      noisy_seed(j) = result(j) + distribution(generator);
    }

    q = noisy_seed;

    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc = tracik_solver.CartToJnt(q, end_effector_pose, result);
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    total_time += elapsed;

    if (rc >= 0)
    {
      // save the result of inverse kinematics
      kinematics::util::write_file(ikin_trac_output_file,  kinematics::util::get_jntarray_as_string(result));
      success++;
    }

    if (int((double)i / num_samples * 100) % 10 == 0)
    ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }

  ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples
  << "\%) with an average of " << total_time / num_samples
  << " secs per sample");
}

int main(int argc, char** argv)
{
  srand(1);
  if ( argc < 2 )
  {
    ROS_FATAL("Don't run this file directly, instead use roslaunch\n\tcommand: roslaunch learn_kinematics compare_ik.launch\n");
    exit(-1);
  }

  ros::init(argc, argv, "compare_ik_node");
  ros::NodeHandle nh;

  // gausian noise in degree
  double noise;

  // read the following parameter provided in launch file
  double timeout;
  std::string chain_start, chain_end, urdf_xml, data_file, out_location;

  nh.param("/kinematics/chain_start", chain_start, std::string(""));
  nh.param("/kinematics/chain_end", chain_end, std::string(""));
  nh.param("/kinematics/urdf_xml", urdf_xml, std::string(""));

  nh.param("/kinematics/timeout", timeout, 0.005);
  nh.param("/kinematics/noise", noise, 0.0);

  nh.param("/kinematics/num_joints", num_joints, 7);
  nh.param("/kinematics/left_joint_offset", left_joint_offset, 0);

  nh.param("/kinematics/joint_array_header", joint_array_header, std::string(""));
  nh.param("/kinematics/ee_pose_header", ee_pose_header, std::string(""));

  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  KDL::Chain chain;
  urdf::Model robot_model = kinematics::util::get_kdl_chain_from_urdf(urdf_xml, chain_start, chain_end, chain);

  KDL::JntArray lower_limit, upper_limit;
  kinematics::util::get_joint_limits(robot_model, chain, lower_limit, upper_limit);

  std::vector<std::vector<double> > data;
  kinematics::util::read_csv(data_file, data);

  test(chain, lower_limit, upper_limit, timeout, data, out_location, noise);

  // Useful when you make a script that loops over multiple launch files that test different robot
  // chains
  std::vector<char*> commandVector;
  commandVector.push_back((char*)"killall");
  commandVector.push_back((char*)"-9");
  commandVector.push_back((char*)"roslaunch");
  commandVector.push_back(NULL);

  char** command = &commandVector[0];
  execvp(command[0], command);

  return 0;
}
