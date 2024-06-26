/**
* util.cpp: This  file contains the utility functions
* Author: Ravi Joshi
* Date: 2016/06/08
*/

#include <util.h>

namespace kinematics {

  /*
  * This function calulcates minimum and maximum limit of each joint in the provided kdl chain
  * Input:
  *   urdf::Model& robot_model = robot model
  *   KDL::Chain& chain = kdl chain of the robot model
  * Output:
  *   KDL::JntArray lower_limit = lower limit of all the joints
  *   KDL::JntArray upper_limit = upper limit of all the joints
  */
  void util::get_joint_limits(const urdf::Model& robot_model, const KDL::Chain& chain, KDL::JntArray& lower_limit, KDL::JntArray& upper_limit)
  {
    std::vector<KDL::Segment> chain_segs = chain.segments;

    boost::shared_ptr<const urdf::Joint> joint;

    std::vector<double> l_bounds, u_bounds;

    lower_limit.resize(chain.getNrOfJoints());
    upper_limit.resize(chain.getNrOfJoints());

    uint joint_num = 0;
    for (unsigned int i = 0; i < chain_segs.size(); ++i)
    {
      joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        joint_num++;
        float lower, upper;
        int hasLimits;
        if (joint->type != urdf::Joint::CONTINUOUS)
        {
          if (joint->safety)
          {
            lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
            upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
          }
          else
          {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else
        {
          hasLimits = 0;
        }
        if (hasLimits)
        {
          lower_limit(joint_num - 1) = lower;
          upper_limit(joint_num - 1) = upper;
        }
        else
        {
          lower_limit(joint_num - 1) = std::numeric_limits<float>::lowest();
          upper_limit(joint_num - 1) = std::numeric_limits<float>::max();
        }
        ROS_INFO_STREAM("Joint name " << joint->name << ", min" << lower_limit(joint_num - 1) << ", max" << upper_limit(joint_num - 1));
      }
    }
  }

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
  urdf::Model util::get_kdl_chain_from_urdf(const std::string& urdf_xml, const std::string& base_link, const std::string& tip_link, KDL::Chain& chain)
  {
    urdf::Model robot_model;
    robot_model.initString(urdf_xml);

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
    {
      ROS_FATAL("Failed to extract kdl tree from xml robot description");
    }

    if(!tree.getChain(base_link, tip_link, chain))
    {
      ROS_FATAL("Couldn't find chain %s to %s",base_link.c_str(),tip_link.c_str());
    }

    return robot_model;
  }

  /*
  * This function reads csv file, which contains floating point numbers
  * Input:
  *   std::string& file_name = absolute file name (full path) of the csv file
  *   int skip_rows = 1 = number of rows to skip while reading file from the top. Default is 1
  *   char delimiter  = ',' = delimiter  separating the cells in file. Default is comma
  * Output:
  *   std::vector<std::vector<double> >& data = 2D vector containing csv data
  */
  void util::read_csv(const std::string& file_name, std::vector<std::vector<double> >& data, int skip_rows, const char delimiter)
  {
    util::read_csv(file_name, data, skip_rows, delimiter );
  }

  /*
  * This function reads csv file, which contains integer numbers
  * Input:
  *   std::string& file_name = absolute file name (full path) of the csv file
  *   int skip_rows = 1 = number of rows to skip while reading file from the top. Default is 1
  *   char delimiter  = ',' = delimiter  separating the cells in file. Default is comma
  * Output:
  *   std::vector<std::vector<int> >& data = 2D vector containing csv data
  */
  void util::read_csv(const std::string& file_name, std::vector<std::vector<int> >& data, int skip_rows, const char delimiter)
  {
    util::read_csv(file_name, data, skip_rows, delimiter );
  }

  /*
  * This template reads csv file
  * Input:
  *   std::string& file_name = absolute file name (full path) of the csv file
  *   int skip_rows = 1 = number of rows to skip while reading file from the top. Default is 1
  *   char delimiter  = ',' = delimiter  separating the cells in file. Default is comma
  * Output:
  *   std::vector<std::vector<T> >& data = 2D vector containing csv data
  */
  template <typename T> void read_csv(const std::string& file_name, std::vector<std::vector<T> >& data, int skip_rows, const char delimiter)
   {
    std::ifstream file(file_name.c_str());

    int i = 0;
    while (i < skip_rows)
    {
      std::string header;
      std::getline(file, header); // skip the header
      i++;
    }
     
    std::string line;
    while (getline(file, line))
    {
      std::vector<T> row;
      std::stringstream ss(line); // we have a line

      while (ss.good())
      {
        std::string substr;
        getline(ss, substr, delimiter ); // split line by comma

        T value;
        std::istringstream(substr) >> value; // convert to T
        row.push_back(value);
      }
      data.push_back(row);
    }
  }

  /*
  * This function write file. Note that, it creates a new file, if it doesn't exists..
  * otherwise it appends.
  * Input:
  *   std::string& file_name = absolute file name (full path)
  *   std::string content = content to write/append
  */
  void util::write_file(const std::string& file_name, const std::string content)
  {
    std::ofstream outfile;
    if(boost::filesystem::exists(file_name))
    {
      outfile.open(file_name.c_str(), std::ios_base::app);
    }
    else
    {
      outfile.open(file_name.c_str());
    }

    outfile << content << std::endl;
    outfile.close();
  }

  /*
  * This function creates string from joint array
  * Input:
  *   KDL::JntArray& joint_conf = joint configuration
  * Returns:
  *   std::string = converted string from input joint array
  */
  std::string util::get_jntarray_as_string(const KDL::JntArray& joint_conf)
  {
    int size = joint_conf.data.size();
    std::string joint_str = "";
    for (int j = 0; j < size - 1; j++)
    joint_str = joint_str + boost::lexical_cast<std::string>(joint_conf.data[j]) + ",";
    joint_str = joint_str + boost::lexical_cast<std::string>(joint_conf.data[size - 1]);

    return joint_str;
  }

  /*
  * This function creates string from frame
  * Input:
  *   KDL::Frame& frame = frame
  * Returns:
  *   std::string = converted string from input frame
  */
  std::string util::get_frame_as_string(const KDL::Frame& frame)
  {
    std::string frame_str = boost::lexical_cast<std::string>(frame.p.data[0]) + ","
    + boost::lexical_cast<std::string>(frame.p.data[1]) + ","
    + boost::lexical_cast<std::string>(frame.p.data[2]);

    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);

    frame_str = frame_str + "," + boost::lexical_cast<std::string>(x) + ","
    + boost::lexical_cast<std::string>(y) + "," + boost::lexical_cast<std::string>(z) + ","
    + boost::lexical_cast<std::string>(w);

    return frame_str;
  }

  /*
  * This function sets jntarray from vector
  * Input:
  *   int index = start index of the joint
  *   int max_joint = maximum number of joints
  *   std::vector<double> &frame_arr = joint configuration represented by array
  * Output:
  *   KDL::JntArray& joint_conf = joint configuration
  */
  void util::get_jntarray_from_vector(int index, int max_joint, const std::vector<double> &frame_arr, KDL::JntArray& joint_conf)
  {
    for(int i = 0; i < max_joint; i++)
    {
      joint_conf(i) = frame_arr[index + i];
    }
  }

  /*
  * This function sets jntarray from standard floating point array
  * Input:
  *   int index = start index of the joint
  *   int max_joint = maximum number of joints
  *   double frame_arr[] = joint configuration represented by array
  * Output:
  *   KDL::JntArray& joint_conf = joint configuration
  */
  void util::get_jntarray_from_array(int index, int max_joint, double frame_arr[], KDL::JntArray& joint_conf)
  {
    for(int i = 0; i < max_joint; i++)
    {
      joint_conf(i) = frame_arr[index + i];
    }
  }

  /*
  * This function sets array from jntarray
  * Input:
  *   KDL::JntArray& joint_conf = joint configuration
  *   int index = start index of the joint
  *   int max_joint = maximum number of joints
  * Output:
  *   std::vector<double>& joint_angles = array of joint angles
  */
  void util::set_array_from_jntarray(int index, int max_joint, const KDL::JntArray& joint_conf, std::vector<double>& joint_angles)
  {
    for (int i = index; i < (index + max_joint); i++)
    {
      joint_angles.at(i) = joint_conf(i - index);
    }
  }

  /*
  * This function sets standard ros floating point array from jntarray
  * Input:
  *   KDL::JntArray& joint_conf = joint configuration
  *   int index = start index of the joint
  *   int max_joint = maximum number of joints
  * Output:
  *   std_msgs::Float64MultiArray& arr = 1D array of joint angles
  */
  void util::set_rosarray_from_jntarray(int index, int max_joint, const KDL::JntArray& joint_conf, std_msgs::Float64MultiArray& arr)
  {
    for (int i = index; i < (index + max_joint); i++)
    {
      arr.data.at(i) = joint_conf(i - index);
    }
  }

  /*
  * This function gets frame from standard floating point array
  * Input:
  *   int index = start index of the joint
  *   double frame_arr[] = joint configuration represented by array
  * Output:
  *   KDL::Frame& frame = frame converted from input array
  */
  void util::get_frame_from_array(int index, double frame_arr[], KDL::Frame& frame)
  {
    double x = frame_arr[index + 0];
    double y = frame_arr[index + 1];
    double z = frame_arr[index + 2];

    frame.p = KDL::Vector (x, y, z);

    double quaternion_x = frame_arr[index + 3];
    double quaternion_y = frame_arr[index + 4];
    double quaternion_z = frame_arr[index + 5];
    double quaternion_w = frame_arr[index + 6];

    frame.M = KDL::Rotation::Quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
  }

  /*
  * This function gets frame from vector
  * Input:
  *   int index = start index of the joint
  *   std::vector<double>& frame_arr = joint configuration represented by vector
  * Output:
  *   KDL::Frame& frame = frame converted from input array
  */
  void util::get_frame_from_vector(int index, const std::vector<double>& frame_arr, KDL::Frame& frame)
  {
    double x = frame_arr[index + 0];
    double y = frame_arr[index + 1];
    double z = frame_arr[index + 2];

    frame.p = KDL::Vector (x, y, z);

    double quaternion_x = frame_arr[index + 3];
    double quaternion_y = frame_arr[index + 4];
    double quaternion_z = frame_arr[index + 5];
    double quaternion_w = frame_arr[index + 6];

    frame.M = KDL::Rotation::Quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
  }

  /*
  * This function convert radian into degree
  * Input:
  *   double radian = angle in radian
  * Returns:
  *   double = converted angle in degree
  */
  double util::degree(double radian)
  {
    double degree = radian * 180 / M_PI;
    return degree ;
  }

  /*
  * This function convert degree into radian
  * Input:
  *   double degree = angle in degree
  * Returns:
  *   double = converted angle in radian
  */
  double util::radian(double degree)
  {
    double radians = degree * M_PI / 180;
    return radians;
  }
}
