/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
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


#ifndef CARTESIAN_TRAJECTORIES_HPP
#define CARTESIAN_TRAJECTORIES_HPP

#include <trac_ik/trac_ik.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>

namespace TRAC_IK_TRAJECTORIES {

  class CartesianTrajectories
  {
  public:
    CartesianTrajectories(const std::string& base_link, const std::string& tip_link, const std::string& robot_description="/robot_description");

    CartesianTrajectories(const KDL::Chain& chain_, const KDL::JntArray& joint_min_, const KDL::JntArray& joint_max_, const std::vector<double>& max_velocities);

    ~CartesianTrajectories();

    int SolveTraj(const KDL::JntArray &q_init, const KDL::Frame &p_in, trajectory_msgs::JointTrajectory& traj, const KDL::Twist& bounds=KDL::Twist::Zero(), bool joint_motion=false);

    inline void SetSolveType(TRAC_IK::SolveType type) {
      solver->SetSolveType(type);
      ROS_DEBUG_STREAM("Changing TRAC_IK CartesianTrajectories to use type "<<(int)type);
    }

    int IsReachable(const KDL::Frame &p_in, const KDL::JntArray seed = KDL::JntArray(0), const KDL::Twist& bounds=KDL::Twist::Zero());

/*
    inline void SetJointMask(const std::vector<bool>& mask) {
      solver->SetJointMask(mask);
      solver2->SetJointMask(mask);
      solver3->SetJointMask(mask);
      reachableSolver->SetJointMask(mask);
    }
*/
    /*
    inline void UpdateOffset(const KDL::Frame& offset) {
      solver->UpdateOffset(offset);
      solver2->UpdateOffset(offset);
      solver3->UpdateOffset(offset);
      chain.segments[chain.segments.size()-1] = KDL::Segment(KDL::Joint(KDL::Joint::None), offset);
    } */
/*
    inline void ClearOffset() {
      solver->ClearOffset();
      solver2->ClearOffset();
      solver3->ClearOffset();
      chain.segments[chain.segments.size()-1] = KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::Identity());
    }
    */

  private:
    KDL::Chain chain;
    KDL::JntArray joint_min, joint_max;
    std::vector<double> joint_velocities;

    int checkPath(trajectory_msgs::JointTrajectory& newTraj, std::vector<KDL::Frame>& frames, std::vector<bool>& validSegment, double stepsize, const KDL::Twist& bounds, bool doInsert);
    double findMaxTime(const std::vector<KDL::JntArray>& traj);

    void findJointNames();

    void printTraj(const std::vector<KDL::JntArray>& arr);

    std::vector<KDL::JntArray> mergeTrajs(const std::vector<KDL::JntArray>& forwards,
                                          const std::vector<KDL::JntArray>& backwards);

    double computeMotion(const std::vector<KDL::JntArray>& traj);

    bool removeRepeats(std::vector<KDL::JntArray>& traj, std::vector<KDL::Frame>& poses);

    trajectory_msgs::JointTrajectory trajTemplate;

    void applyVelocityConstraints(const trajectory_msgs::JointTrajectory& newTraj, std::vector<double>& timediff); 

    void updateTrajectory(trajectory_msgs::JointTrajectory& newTraj, const std::vector<double>& timediff);

    boost::scoped_ptr<TRAC_IK::TRAC_IK> solver, solver2, solver3, reachableSolver;

    int doSolver(const KDL::JntArray &q_init, const KDL::Frame &p_in, trajectory_msgs::JointTrajectory& traj, const KDL::Twist& bounds, bool joint_motion);

    KDL::JntArray q_nom;

    inline bool EqualVecs(const std::vector<double>& arr1, const std::vector<double>& arr2) {
      double err = 0;
      for (uint i=0; i<arr1.size(); i++) {
        err = std::max(err, std::abs(arr1[i] - arr2[i]));
      }
     
      return (err < std::numeric_limits<float>::epsilon());
    }


    int insertSteps(trajectory_msgs::JointTrajectory& traj, 
                    std::vector<KDL::Frame>& frames,
                    std::vector<bool>& validSegment,
                    int steps,
                    uint curr_index, const KDL::Twist& bounds);

    boost::posix_time::ptime solveStart;
    
  };

}

#endif
