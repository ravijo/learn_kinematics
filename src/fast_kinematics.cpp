/**
* fast_kinematics.cpp: class file designed to use trac_ik
* Author: Ravi Joshi
* Date: 2016/06/08
*/

#include <fast_kinematics.h>

namespace kinematics {

  /*
  * Constructor of fast_kinematics class
  * Input:
  *   KDL::Chain& chain= kdl chain
  *   KDL::JntArray& q_min = minimum joint limits
  *   KDL::JntArray& q_max = maximum joint limits
  *   double timeout = timeout for ik solver in seconds
  */
  fast_kinematics::fast_kinematics(const KDL::Chain& chain, const KDL::JntArray& lower_limit, const KDL::JntArray& upper_limit, double timeout)
  {
    ik_solver = new TRAC_IK::TRAC_IK(chain, lower_limit, upper_limit, timeout, eps);
    init();
  }

  void fast_kinematics::init()
  {
    valid = ik_solver->getKDLChain(chain);

    if (!valid)
    {
      ROS_ERROR("There was no valid KDL chain found");
      return;
    }

    valid = ik_solver->getKDLLimits(ll, ul);

    if (!valid)
    {
      ROS_ERROR("There were no valid KDL joint limits found");
      return;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    ROS_INFO("Using %d joints", chain.getNrOfJoints());

    fk_solver = new KDL::ChainFkSolverPos_recursive(chain);// Set up KDL FK
  }

  /*
  * Constructor of fast_kinematics class
  * Input:
  *   std::string& urdf_param = URDF parameter to represent KDL structure
  *   std::string& chain_start = Start point in KDL chain structure
  *   std::string& chain_end = End point in KDL chain structure
  *   double timeout = timeout for ik solver in seconds
  */
  fast_kinematics::fast_kinematics(std::string& urdf_param, std::string& chain_start, std::string& chain_end, double timeout)
  {
    // This constructor parses the URDF loaded in rosparm urdf_param into the needed KDL structures
    ik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps);
    init();
  }

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
  int fast_kinematics::forward_kinematics(const KDL::JntArray& joint_conf, KDL::Frame& ee_pose)
  {
    int result = fk_solver->JntToCart(joint_conf, ee_pose);
    return result;
  }

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
  int fast_kinematics::inverse_kinematics(const KDL::Frame& ee_pose, KDL::JntArray& joint_conf, const KDL::JntArray& joint_seed)
  {
    int result = ik_solver->CartToJnt(joint_seed, ee_pose, joint_conf);
    return result;
  }

  /*
  * This function creates nominal chain configuration midway between all joint limits
  * Output:
  *   KDL::JntArray& joint_conf = nominal chain configuration
  */
  void fast_kinematics::get_nominal_joint_conf(KDL::JntArray& joint_conf)
  {
    for (uint j=0; j<joint_conf.data.size(); j++) {
      joint_conf(j) = (ll(j)+ul(j))/2.0;
    }
  }

  fast_kinematics::~fast_kinematics()
  {
    //todo
  }
}
