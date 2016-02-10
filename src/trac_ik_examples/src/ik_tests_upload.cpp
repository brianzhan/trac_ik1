/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/
// for testing, moved ~/trac_ik/src/ada_description/robots/mico-modified_nolimits back by 1 directory
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/nlopt_ik.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <limits>
#include <boost/date_time.hpp>
#include <trac_ik/dual_quaternion.h>

// NOTE: to build, modified trac_ik/build/CMakeFiles/2.8.12.2$/CMakeCXXCompiler.cmake to include set(CMAKE_CXX_FLAGS "--std=c++11")

// no longer needed, here just for testing
double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void cartesianToAngle(ros::NodeHandle& nh, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, double translations[], double angleQuaternion[]) {
  double eps = 1e-5;
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
  KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  KDL::JntArray joint_seed;
  KDL::Frame desired_end_effector_pose;
  KDL::JntArray return_joints;

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  std::cout << "ll data size is " << ll.data.size() << std::endl;


  ROS_INFO ("Using %d joints",chain.getNrOfJoints());
  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK) 

  // Create Nominal chain configuration midway between all joint limits
  
  KDL::JntArray nominal(chain.getNrOfJoints());
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());



  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;

  end_effector_pose.M.Quaternion(angleQuaternion[0], angleQuaternion[1], angleQuaternion[2], angleQuaternion[3]);
  end_effector_pose.p.x(translations[0]);
  end_effector_pose.p.y(translations[1]);
  end_effector_pose.p.z(translations[2]);

  start_time = boost::posix_time::microsec_clock::local_time();
  int rc=tracik_solver.CartToJnt(nominal, end_effector_pose, result); // TO DO:: ADD IN BOUNDS PARAMETER

  if (rc < 0) {
    std::cout << "Error RC below 0" << std::endl;
  } else {
    for(int i = 0; i < result.rows(); i++) {
      for(int j = 0; j < result.columns(); j++) {
        std::cout << "joint angle[" << i << "][" << j << "] = " << result(i, j) << std::endl;
      }
    }
    std::cout << "CartToJnt completed" << std::endl;
  }
}  

void angleToCartesian(ros::NodeHandle& nh, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, double angleInputs[])
{
  double eps = 1e-5;
  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against thef KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);
  /* // archived code from debugging
  for (uint i=0; i < 6; i++) {
    std::cout << "lower bound " << i << " is " << ll(i) << std::endl;
    std::cout << "upper bound " << i << " is " << ul(i) << std::endl;
  }
  */
  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  std::cout << "ll data size is " << ll.data.size() << std::endl;

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());

  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK) 

  // Create Nominal chain configuration midway between all joint limits
  
  KDL::JntArray nominal(chain.getNrOfJoints());
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());


  for (uint i=0; i < num_samples; i++) { // once, here in case multiple paths tested
    for (uint j=0; j<ll.data.size(); j++) {
      q(j)= angleInputs[j]; 
      // std::cout << "angle input is " << q(j) << std::endl; // taken out because print of debugging info
    }    
    JointList.push_back(q); // JointList gets initialized
  }

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;

  double total_time=0;

  total_time=0;
// start of relevant code
  ROS_INFO_STREAM("*** Testing TRAC-IK with "<<num_samples<<" random samples");
  std::cout << "number of samples is " << num_samples << std::endl;
  for (uint i=0; i < num_samples; i++) {
    fk_solver.JntToCart(JointList[i],end_effector_pose);
    std::cout << end_effector_pose.p.x() << "\n";

    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result); // I don't really see the point in diong CartToJnt right here
   // std::cout << result << std::endl;

    //ROS_INFO_STREAM("End effector is " << end_effector << " ");

    if (rc>=0) {
        for (int j = 0; j  < 6; j++) { // FIX THIS LATER
          std::cout << "joint velocity[" << j << "]=" << result(j) << std::endl;
        }
        math3d::matrix3x3<double> currentRotationMatrix(end_effector_pose.M.data);
        double x, y, z, w;
        end_effector_pose.M.GetQuaternion(x, y, z, w);
        std::cout << "Quaternion x : " << x << std::endl;
        std::cout << "Quaternion y : " << y << std::endl;
        std::cout << "Quaternion z : " << z << std::endl;
        std::cout << "Quaternion w : " << w << std::endl;



        math3d::quaternion<double> currentQuaternion = math3d::rot_matrix_to_quaternion<double>(currentRotationMatrix);
        std::cout << "Quaternion is " << currentQuaternion << std::endl;
        std::cout << " result eefx is " << end_effector_pose.p.x() << std::endl;
        std::cout << " result eefy is " << end_effector_pose.p.y() << std::endl; 
        std::cout << " result eefz is " << end_effector_pose.p.z() << std::endl << std::endl;
    }
    else {
        std::cout << "error " << std::endl;
    }

    // math3d::matrix3x3<double> currentRotationMatrix(currentPose.M.data); 
   // math3d::quaternion<double> currentQuaternion = math3d::rot_matrix_to_quaternion<double>(currentRotationMatrix);
    //math3d::point3d currentTranslation (currentPose.p.data);
  double x1, x2, y1, y2, z1, z2;
  /*
    x1 = end_effector_pose.M.UnitX().x();
    if (int((double)i/num_samples*100)%10 == 0)
      ROS_INFO_STREAM_THROTTLE(1,int((i)/num_samples*100)<<"\% done");*/
    }
}





int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("num_samples", num_samples, 1);
  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));
  std::cout << "chain start is " << chain_start << std::endl;
  std::cout << "chain end is " << chain_end << std::endl;
  num_samples = 1;
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  // ***********************************start to angleToCartesian**************************************

  double angleInputs[] = {0.232444, 0.486403, -0.959714, -2.12114, 2.56199, -1.96953, -2.39557};
  angleToCartesian(nh, num_samples, chain_start, chain_end, timeout, urdf_param, angleInputs); // NEED TO DO: output x y z as 


  // **********************************start of CartesianToAngle

  double translations[] = {0.316787, 0.117263, 0.0302719};
  double rotationQuaternion[] = {0.479594, 0.833593, -0.0911262, 0.258475};

  //cartesianToAngle(nh, chain_start, chain_end, timeout, urdf_param, translations, rotationQuaternion);

  // Useful when you make a script that loops over multiple launch files that test different robot chains
  std::vector<char *> commandVector;
  commandVector.push_back((char*)"killall");
  commandVector.push_back((char*)"-9");
  commandVector.push_back((char*)"roslaunch");
  commandVector.push_back(NULL);  

  char **command = &commandVector[0];
  execvp(command[0],command);

  return 0;
}

