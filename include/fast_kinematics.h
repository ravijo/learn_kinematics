/**
* fast_kinematics.h: header file designed to use trac_ik
* Author: Ravi Joshi
* Date: 2016/06/08
*/

#ifndef FAST_KINEMATICS_H_
#define FAST_KINEMATICS_H_

#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

namespace kinematics {
  class fast_kinematics {
  public:

    /*
    * Constructor of fast_kinematics class
    * Input:
    *   KDL::Chain& chain= kdl chain
    *   KDL::JntArray& lower_limit = minimum joint limits
    *   KDL::JntArray& upper_limit = maximum joint limits
    *   double timeout = timeout for ik solver in seconds
    */
    fast_kinematics(const KDL::Chain& chain, const KDL::JntArray& lower_limit, const KDL::JntArray& upper_limit, double timeout);

    /*
    * Constructor of fast_kinematics class
    * Input:
    *   std::string& urdf_param = URDF parameter to represent KDL structure
    *   std::string& chain_start = Start point in KDL chain structure
    *   std::string& chain_end = End point in KDL chain structure
    *   double timeout = timeout for ik solver in seconds
    */
    fast_kinematics(std::string& urdf_param, std::string& chain_start, std::string& chain_end, double timeout);

    ~fast_kinematics();

    /*
    * This function calulcates forward kinematics
    * It is done using KDL ChainFkSolverPos_recursive  solver
    * Input:
    *   KDL::JntArray& joint_conf = Joint configuration containing all join angles in radians
    * Output:
    *   KDL::Frame& ee_pose = End-effector pose containing positon and orientation
    * Returns
    *   >= 0  : in case of success
    *   negative number : otherwise
    */
    int forward_kinematics(const KDL::JntArray& joint_conf, KDL::Frame& ee_pose);

    /*
    * This function calulcates inverse kinematics
    * It is done using TRAC_IK  solver
    * It runs for the full timeout then returns the solution that minimizes SSE from the seed
    * Input:
    *   KDL::Frame& ee_pose = End-effector pose containing positon and orientation
    *   KDL::JntArray& joint_seed = approximated joint configuration
    * Output:
    *   KDL::JntArray& joint_conf = Joint configuration corresponding to input frame
    * Returns
    *   >= 0  : in case of success
    *   negative number : otherwise
    */
    int inverse_kinematics(const KDL::Frame& ee_pose, KDL::JntArray& joint_conf, const KDL::JntArray& joint_seed);

    /*
    * This function creates nominal chain configuration midway between all joint limits
    * Output:
    *   KDL::JntArray& joint_conf = nominal chain configuration
    */
    void get_nominal_joint_conf(KDL::JntArray& joint_conf);

  private:
    void init();

    static constexpr double eps = 1e-5;

    bool valid;

    KDL::Chain chain;
    KDL::JntArray ll, ul; // lower joint limits, upper joint limits
    TRAC_IK::TRAC_IK* ik_solver;
    KDL::ChainFkSolverPos_recursive* fk_solver;
  };
}

#endif				/* FAST_KINEMATICS_H_ */
