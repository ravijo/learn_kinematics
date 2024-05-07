# learn_kinematics
Forward and Inverse Kinematics of Baxter Robot

~~**PLEASE ADD 0.01 TO Z COORDINATE BEFORE CALCULATING IK. SEE LINE 25 IN [ik_test.py](scripts/ik_test.py#L25) FOR MORE INFO**~~

## Dependencies
* ~~[moveit!](http://moveit.ros.org/)~~
* ~~[baxter_pykdl](https://github.com/RethinkRobotics/baxter_pykdl)~~
* [trac_ik](https://bitbucket.org/traclabs/trac_ik)

## Install trac_ik
1. Open terminal or press <kbd>CTRL</kbd>+<kbd>ALT</kbd>+<kbd>T</kbd>
1. Run following command
  ```console
  sudo apt-get install ros-${ROS_DISTRO}-trac-ik
  ```
  For example, in the case of ROS Indigo, the following command is equivalent to the one above: `sudo apt-get install ros-indigo-trac-ik`

## Installation
* ~~Install baxter_pykdl package by downloading the package from [git](https://github.com/RethinkRobotics/baxter_pykdl) into the workspace src folder and running `catkin_make`.~~
* ~~Install moveit dependecies: `sudo apt-get install ros-indigo-moveit-ros`~~
* ~~Uncompress the downloaded trac_ik archive from the [website](https://bitbucket.org/traclabs/trac_ik)~~
* ~~Copy all four directories *trac_ik*, *trac_ik_example*, *trac_ik_kinematics_plugin* and *trac_ik_lib* into ros workspace~~
* ~~Run `catkin_make` command from ros workspace~~
* ~~If the abpve command shows following error- `fatal error: nlopt.hpp: No such file or directory #include <nlopt.hpp>`
 please run `sudo apt-get install libnlopt-dev`~~
* ~~Run again `catkin_make` command~~

* Download or clone the repository
* Go to the ros workspace, i.e., `cd ros_ws/`
* Run `catkin_make`
* If the abpve command shows following error- `fatal error: learn_kinematics/fast_kinematics_service.h: No such file or directory`, please run `catkin_make` again

## Programs
### fkin.py, ikin.py, fkin2.py
These programs use the baxter_pykdl package which requires the robot urdf description to perform IK or FK.

*Please connect to Baxter robot or run the baxter simulator for these programs to work properly.*

### fast_kinematics_pub.cpp
This program listen to rostopic `fast_kinematics_listener` for receiving end-effector pose from user. Later on it performs inverse kinematic analysis and publishs the result to another rostopic `fast_kinematics_publisher`

### fast_kinematics_srv.cpp
This is a service implementaion for kinematic analysis. It creates a service `fast_kinematics_service`, which perfroms inverse kinematic analysis on request from client.

## How to use
### fast_kinematics_pub.cpp
* First, you need to publish `std_msgs::Float64MultiArray` containing end-effector pose (position in cartesian system and orientaion in quaternion system) to the rostopic `fast_kinematics_listener` as input.
* Now, run the subscriber and publisher by executing following command `roslaunch learn_kinematics fast_kinematics_pub.launch`
* The output joint angles can be seen by following command `rostopic echo fast_kinematics_publisher`

*You may notice some error initially, as the subscriber haven't receieved any input and publisher starts publishing default values causing this error* 

### fast_kinematics_srv.cpp
* Run the service by following command `roslaunch learn_kinematics fast_kinematics_srv.launch`
* Run the service client, which calls `fast_kinematics_service` with `learn_kinematics::fast_kinematics_service` service data type. Currently, it is done in c++ and python in following way -
 * __C++:__ Run the client by following command `rosrun learn_kinematics fast_kinematics_srv_test`
 * __Python:__ Run the client by following command `rosrun learn_kinematics fast_kinematics_srv_test.py`
