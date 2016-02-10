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

#include <trac_ik_trajectories/cartesian_trajectories.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <memory>

namespace TRAC_IK_TRAJECTORIES {

#define end_time 0.1
#define iterative_time 0.001
#define eps 1e-3
#define timeout 2.0

  CartesianTrajectories::CartesianTrajectories(const std::string& base_link, const std::string& tip_link, const std::string& robot_description ) {
    ros::NodeHandle node_handle("~");

    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,robot_description);
    node_handle.searchParam(urdf_xml,full_urdf_xml);
    
    ROS_DEBUG_NAMED("trac_ik","Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
        ROS_FATAL_NAMED("trac_ik","Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
      }
    
    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);
    
    ROS_DEBUG_STREAM_NAMED("trac_ik","Reading joints and links from URDF");
    
    KDL::Tree tree;
    
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
      ROS_FATAL("Failed to extract kdl tree from xml robot description");
      return;
    }

    if(!tree.getChain(base_link, tip_link, chain)) {
      ROS_FATAL("Couldn't find chain %s to %s",base_link.c_str(),tip_link.c_str());
      return;
    }

    std::vector<KDL::Segment> chain_segs = chain.segments;

    boost::shared_ptr<const urdf::Joint> joint;

    std::vector<double> l_bounds, u_bounds;

    joint_min.resize(chain.getNrOfJoints());
    joint_max.resize(chain.getNrOfJoints());

    uint joint_num=0;
    for(unsigned int i = 0; i < chain_segs.size(); ++i) {
          
      joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
        joint_num++;
        float lower, upper;
        int hasLimits;
        joint_velocities.push_back(std::abs(joint->limits->velocity)); //required in URDF
        if ( joint->type != urdf::Joint::CONTINUOUS ) {
          if(joint->safety) {
            lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
            upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
          } else {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else {
          hasLimits = 0;
        }
        if(hasLimits) {
          joint_min(joint_num-1)=lower;
          joint_max(joint_num-1)=upper;
        }
        else {
          joint_min(joint_num-1)=std::numeric_limits<float>::lowest();
          joint_max(joint_num-1)=std::numeric_limits<float>::max();
        }
        ROS_INFO_STREAM("Cartesian trajectories using joint "<<chain_segs[i].getName()<<" "<<joint_min(joint_num-1)<<" "<<joint_max(joint_num-1));
      }
    }

    findJointNames();

    q_nom.resize(joint_min.data.size());
    for (uint i=0; i< joint_min.data.size(); i++)
      q_nom(i) = (joint_min(i)+joint_max(i))/2.0;

    solver.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, end_time, eps, TRAC_IK::Manip1));

    solver2.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, iterative_time, eps, TRAC_IK::Distance));

    solver3.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, end_time, eps, TRAC_IK::Distance));

    reachableSolver.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, end_time, eps)); //Speed Type

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::Identity()));

  }

  CartesianTrajectories::CartesianTrajectories(const KDL::Chain& chain_, const KDL::JntArray& joint_min_, const KDL::JntArray& joint_max_, const std::vector<double>& max_velocities) : chain(chain_), joint_min(joint_min_), joint_max(joint_max_), joint_velocities(max_velocities) {
    
    assert (joint_min.data.size() == chain.getNrOfJoints());
    assert (joint_max.data.size() == chain.getNrOfJoints());
    assert (joint_velocities.size() == chain.getNrOfJoints());
    
    findJointNames();

    q_nom.resize(joint_min.data.size());
    for (uint i=0; i< joint_min.data.size(); i++)
      q_nom(i) = (joint_min(i)+joint_max(i))/2.0;

    solver.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, end_time, eps, TRAC_IK::Manip1));

    solver2.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, iterative_time, eps, TRAC_IK::Distance));

    solver3.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, end_time, eps, TRAC_IK::Distance));

    reachableSolver.reset(new TRAC_IK::TRAC_IK(chain, joint_min, joint_max, end_time, eps)); //Speed Type

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::Identity()));

  }

  int CartesianTrajectories::IsReachable(const KDL::Frame &p_in, const KDL::JntArray seed, const KDL::Twist& bounds) {
    KDL::JntArray q_out;
    int rc;
    if (seed.data.size()==0)
      rc = reachableSolver->CartToJnt(q_nom, p_in, q_out, bounds);
    else {
      assert(seed.data.size()==q_nom.data.size());
      rc = reachableSolver->CartToJnt(seed, p_in, q_out, bounds);
    }
    return rc;
  }

  void CartesianTrajectories::findJointNames() {
    uint segments = chain.getNrOfSegments();
    trajTemplate.joint_names.clear();
    
    for (uint i=0; i<segments; i++) {
      KDL::Joint jnt = chain.getSegment(i).getJoint();
      if (jnt.getType()!=KDL::Joint::None) {
        trajTemplate.joint_names.push_back(jnt.getName());
      }
    }
  }
  

  CartesianTrajectories::~CartesianTrajectories() {}


  void CartesianTrajectories::printTraj(const std::vector<KDL::JntArray>& arr) {
    for (uint i=0; i< arr.size(); i++) {
      for (uint j=0; j<arr[i].data.size(); j++)
        std::cerr << arr[i](j)<<" ";
      if (i > 0)
        std::cerr << TRAC_IK::TRAC_IK::JointErr(arr[i],arr[i-1]);
      std::cerr <<"\n";
    }
    std::cerr << computeMotion(arr)<<"\n\n";
  }
  


  int CartesianTrajectories::SolveTraj(const KDL::JntArray &q_init, const KDL::Frame &p_in, trajectory_msgs::JointTrajectory& traj, const KDL::Twist& bounds,
                                       bool joint_motion) {

    assert(q_init.data.size()==trajTemplate.joint_names.size());

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    solveStart = start_time;

    boost::posix_time::time_duration timediff;
   
    trajectory_msgs::JointTrajectory distTraj;

    int rc = doSolver(q_init, p_in, distTraj, bounds, joint_motion);

    if (rc < 100) 
      ROS_WARN("Trajectory search failed");
    else 
      traj = distTraj;
  
    
    
    timediff = boost::posix_time::microsec_clock::local_time()-start_time;
    ROS_WARN_STREAM("Took "<<timediff.total_nanoseconds()/1000000000.0<<" seconds to complete trajectory search");
    
    return rc;
  }

  int CartesianTrajectories::doSolver(const KDL::JntArray &q_init, const KDL::Frame &p_in, trajectory_msgs::JointTrajectory& traj, const KDL::Twist& bounds, bool joint_motion) {

    if ((boost::posix_time::microsec_clock::local_time()-solveStart).total_nanoseconds()/1e9 > timeout)
      return 0;
    
    KDL::JntArray q_out;

    std::vector<KDL::JntArray> solutions;

    traj = trajTemplate;

    int rc;
    
    rc = solver->CartToJnt(q_init, p_in, solutions);
    if (rc < 0)
      rc = solver->CartToJnt(q_init, p_in, solutions, bounds);
    if (rc < 0)
      return rc;

    trajectory_msgs::JointTrajectory newTraj=trajTemplate;

    bool doInsert;

    KDL::Frame p_start;
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    fksolver.JntToCart(q_init,p_start);

    for (uint samples=0; samples<solutions.size(); samples++) {

      if ((boost::posix_time::microsec_clock::local_time()-solveStart).total_nanoseconds()/1e9 > timeout)
        return 0;

      
      //      ROS_INFO("***Looking for new trajectory***");
      
      q_out = solutions[samples];

      newTraj.points.clear();
      doInsert=true;
      
      trajectory_msgs::JointTrajectoryPoint pnt;

      std::vector<double> jnts(q_init.data.size());
      for (uint j=0; j<jnts.size(); j++)
        jnts[j] = q_init(j);
      pnt.positions = jnts;
      newTraj.points.push_back(pnt);
      
      /////////////////////////////////////////////////////
      // First see if we need to move at all
      
      KDL::JntArray sub(q_out.data.size());
      Subtract(q_out,q_init,sub);
      
      if (sub.data.isZero(std::numeric_limits<float>::epsilon())) {
        ROS_INFO("No trajectory needed to get from joint seed to requested pose");
        traj = newTraj;
        return 100;
      }
      /////////////////////////////////////////////////////
      
      
      /////////////////////////////////////////////////////
      // Next start a coarse discretization
      
      for (uint j=0; j<jnts.size(); j++)
        jnts[j] = q_out(j);
      pnt.positions = jnts;
      newTraj.points.push_back(pnt);

      std::vector<double> times;
      applyVelocityConstraints(newTraj,times);
      updateTrajectory(newTraj,times);

      if (joint_motion) {
        ROS_INFO("Joint motion found");
        traj=newTraj;
        return 100;
      }

      std::vector<KDL::Frame> frames;
      frames.push_back(p_start);
      frames.push_back(p_in);

      std::vector<bool> validSegment(newTraj.points.size()-1,false);

      for (double stepsize=0.1; stepsize>=1e-3; stepsize/=10.0) {
        rc = checkPath(newTraj, frames, validSegment, stepsize, bounds, doInsert);

        if (rc < 100) {
          //ROS_WARN*****************
          break;
        }
      }
      
      if (rc == 100) {
        doInsert=false;
        rc = checkPath(newTraj, frames, validSegment, 1e-3, bounds, doInsert);
        if (rc == 100)  {
          traj=newTraj;
          break;      
        }
      }
    }

    return rc;
  }

  int CartesianTrajectories::checkPath(trajectory_msgs::JointTrajectory& newTraj, std::vector<KDL::Frame>& frames, std::vector<bool>& validSegment, double stepsize, const KDL::Twist& bounds, bool doInsert) {
        
    KDL::ChainFkSolverPos_recursive fksolver(chain);

    //    solver2->SetTimeout(std::min(iterative_time,stepsize));

    for (uint i=1; i < newTraj.points.size(); i++) {
      if ((boost::posix_time::microsec_clock::local_time()-solveStart).total_nanoseconds()/1e9 > timeout)
        return 0;

      if (validSegment[i-1])
        continue;
      trajectory_msgs::JointTrajectoryPoint curr_pnt = newTraj.points[i-1];
      trajectory_msgs::JointTrajectoryPoint next_pnt = newTraj.points[i];
      double delta_time = (next_pnt.time_from_start-curr_pnt.time_from_start).toSec();
      int steps = ceil(delta_time/stepsize);
      KDL::JntArray jnts(next_pnt.positions.size());

      KDL::Twist delta_pose = diff(frames[i-1],frames[i]);

      std::vector<double> delta_jnt;
      for (uint z=0; z<next_pnt.positions.size(); z++) 
        delta_jnt.push_back(next_pnt.positions[z]-curr_pnt.positions[z]);

      bool good = true;
      for (int j=1; j<steps; j++) {
        double percentage = j/(double)steps;
        for (uint z=0; z<next_pnt.positions.size(); z++) 
          jnts(z) = curr_pnt.positions[z]+(delta_jnt[z]*percentage);       

        KDL::Frame new_pose;
        fksolver.JntToCart(jnts,new_pose);
        
        KDL::Frame expected_pose = addDelta(frames[i-1],delta_pose,percentage);
      
        KDL::Twist delta_twist = diffRelative(expected_pose, new_pose);
        
        if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
          delta_twist.vel.x(0);
        
        if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
          delta_twist.vel.y(0);
        
        if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
          delta_twist.vel.z(0);
        
        if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
          delta_twist.rot.x(0);
        
        if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
          delta_twist.rot.y(0);
        
        if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
          delta_twist.rot.z(0);

        if(!Equal(delta_twist,KDL::Twist::Zero(),eps)) {
          good = false;
          break;
        }
      }
      
      if (!good) {
        if (doInsert) {
          uint prev_size = frames.size();
          //          ROS_INFO_STREAM("Inserting fine grained at segment "<<i<<" of "<<frames.size()-1);
          int rc = insertSteps(newTraj, frames, validSegment, steps, i-1, bounds);
          if (rc < 100)
            return rc;
          uint new_size = frames.size();
          i+=new_size-prev_size;
        }
        else 
          return 100.0*i/newTraj.points.size();
      }
      else // Good segment
        if (steps > 1) // actually examined segment
          validSegment[i-1]=true;
    }
    
    if (!doInsert)
      ROS_INFO_STREAM("Final trajectory size is "<<newTraj.points.size()<<" steps");
    //    else 
    //      ROS_INFO_STREAM("New trajectory size is "<<newTraj.points.size()<<" steps at time resolution "<<stepsize);

    std::vector<double> times;
    applyVelocityConstraints(newTraj,times);
    updateTrajectory(newTraj,times);

    return 100;
  }

  int CartesianTrajectories::insertSteps(trajectory_msgs::JointTrajectory& traj, 
                                         std::vector<KDL::Frame>& frames,
                                         std::vector<bool>& validSegment,
                                         int steps,
                                         uint curr_index,
                                         const KDL::Twist& bounds) {

    // boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    // boost::posix_time::time_duration timediff;


    KDL::Twist forwards_delta_pose = diff(frames[curr_index],frames[curr_index+1]);
    KDL::Twist backwards_delta_pose = diff(frames[curr_index+1],frames[curr_index]);

    trajectory_msgs::JointTrajectoryPoint curr_pnt = traj.points[curr_index];
    trajectory_msgs::JointTrajectoryPoint next_pnt = traj.points[curr_index+1];

    // std::vector<double> delta_jnt;
    // for (uint z=0; z<next_pnt.positions.size(); z++) 
    //   delta_jnt.push_back(next_pnt.positions[z]-curr_pnt.positions[z]);

    std::vector<KDL::JntArray> forwards, backwards;

    KDL::JntArray start_jnt(curr_pnt.positions.size());
    KDL::JntArray end_jnt(curr_pnt.positions.size());
    KDL::JntArray delta_jnt(curr_pnt.positions.size());

    for(uint i=0; i< start_jnt.data.size(); i++) {
      start_jnt(i) = curr_pnt.positions[i];
      end_jnt(i) = next_pnt.positions[i];
      delta_jnt(i) = next_pnt.positions[i]-curr_pnt.positions[i];
    }

    forwards.push_back(start_jnt);
    backwards.push_back(end_jnt);
    
    std::vector<KDL::Frame> new_frames;
    new_frames.push_back(frames[curr_index]);

    int rc = -3;
    KDL::JntArray q_out;

    for (uint i = 1; i<steps; ++i) {

      if ((boost::posix_time::microsec_clock::local_time()-solveStart).total_nanoseconds()/1e9 > timeout)
        return 0;


      double percentage = i / (double) steps;
      
      KDL::Frame expected_pose = addDelta(frames[curr_index],forwards_delta_pose,percentage);       

      KDL::JntArray seed(start_jnt.data.size());
      for (uint z=0; z<seed.data.size(); z++) 
        seed(z) = start_jnt(z)+(delta_jnt(z)*percentage);

      uint retries=0;
      do {
        if (retries==0)
          rc = solver2->CartToJnt(seed, expected_pose, q_out);
        else if (retries==1)
          rc = solver2->CartToJnt(seed, expected_pose, q_out, bounds);
        retries++;
      } while (rc < 0 && retries < 2);

      if (rc < 0) {
        int frac = 100.0*curr_index/frames.size();
        //        ROS_WARN_STREAM("Trajectory failed at "<<frac<<"\% way through.");
        return frac;
      }

      forwards.push_back(q_out);
      new_frames.push_back(expected_pose);
    } 
    
    forwards.push_back(backwards[0]);
    new_frames.push_back(frames[curr_index+1]);

    // for (uint i = 1; i<steps; ++i) {
    //   double percentage = i / (double) steps;
      
    //   KDL::Frame expected_pose = addDelta(frames[curr_index+1],backwards_delta_pose,percentage);

    //   KDL::JntArray seed(start_jnt.data.size());
    //   for (uint z=0; z<seed.data.size(); z++) 
    //     seed(z) = end_jnt(z)-(delta_jnt(z)*percentage);
    //   uint retries = 0;
    //   do {
    //     if (retries==0)
    //       rc = solver2->CartToJnt(seed, expected_pose, q_out);
    //     else if (retries==1)
    //       rc = solver2->CartToJnt(seed, expected_pose, q_out, bounds);
    //     retries++;
    //   } while (rc < 0 && retries < 2);
      
    //   if (rc < 0) {
    //     int frac = 100.0*curr_index/frames.size();
    //     //        ROS_WARN_STREAM("Trajectory failed "<<frac<<"\% way through.");
    //     return frac;
    //   }

    //   backwards.push_back(q_out);
    // } 

    // backwards.push_back(forwards[0]);

    // std::reverse(backwards.begin(),backwards.end());
    
    std::vector<KDL::JntArray> kdl_traj;

    kdl_traj = forwards;//mergeTrajs(forwards,backwards);

    removeRepeats(kdl_traj, new_frames);

    // if (forwards.size() != kdl_traj.size()) {
    //   ROS_INFO_STREAM("Expecting to insert "<<forwards.size()-2<<" new points");      
    //   ROS_INFO_STREAM("Only inserted "<<kdl_traj.size()-2<<" new points");
    // }

    std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
    trajectory_msgs::JointTrajectoryPoint newPnt;
    newPnt.positions.resize(kdl_traj[0].data.size());
    
    for (uint i = 1; i < kdl_traj.size() - 1; i++) {
      for (uint z=0; z<newPnt.positions.size(); z++) 
        newPnt.positions[z] = kdl_traj[i](z);
      new_traj.push_back(newPnt);
    }

    std::vector<bool> newSegs(new_traj.size(),false);
    
    traj.points.insert(traj.points.begin()+curr_index+1,new_traj.begin(), new_traj.end());
    frames.insert(frames.begin()+curr_index+1,new_frames.begin()+1,new_frames.end()-1);
    validSegment.insert(validSegment.begin()+curr_index+1,newSegs.begin(),newSegs.end());

    assert(traj.points.size() == frames.size());
    assert(validSegment.size() == frames.size()-1);

    //    timediff = boost::posix_time::microsec_clock::local_time()-start_time;
    //    ROS_WARN_STREAM("insertSteps took "<<timediff.total_nanoseconds()/1000000000.0<<" seconds");


    
    return 100;

}


  bool CartesianTrajectories::removeRepeats(std::vector<KDL::JntArray>& traj,
                                            std::vector<KDL::Frame>& poses) {
    assert(traj.size()>0);

    std::vector<KDL::JntArray> new_traj;
    std::vector<KDL::Frame> new_poses;

    new_traj.push_back(traj[0]);
    new_poses.push_back(poses[0]);
    KDL::JntArray sub(traj[0].data.size());

    for (uint i=1; i< traj.size(); i++) {
      Subtract(traj[i],traj[i-1],sub);
      if (!sub.data.isZero(std::numeric_limits<float>::epsilon())) {
        new_traj.push_back(traj[i]);
        new_poses.push_back(poses[i]);
      }
    }
    
    if (new_traj.size() != traj.size()) {
      traj = new_traj;
      poses = new_poses;
      return true;
    }
    return false;
  }

  double CartesianTrajectories::computeMotion(const std::vector<KDL::JntArray>& traj) {
    double err = 0;

    for (uint i=1; i < traj.size(); i++)
      err += std::sqrt(TRAC_IK::TRAC_IK::JointErr(traj[i],traj[i-1]));
    
    return err;
  }

  std::vector<KDL::JntArray>
  CartesianTrajectories::mergeTrajs(const std::vector<KDL::JntArray>& forwards,
                                    const std::vector<KDL::JntArray>& backwards) {

    assert(forwards.size()==backwards.size());

    std::vector<KDL::JntArray> merged_arr = backwards;
    double min_err = findMaxTime(merged_arr);
    //int index = 1;

    for (uint i=2; i< backwards.size(); i++) {
      std::vector<KDL::JntArray> new_arr(forwards.begin(),forwards.begin()+i);
      new_arr.insert(new_arr.end(),backwards.begin()+i,backwards.end());
      assert(new_arr.size() == backwards.size());
      double curr_err = findMaxTime(new_arr);
      if (curr_err < min_err) {
        min_err = curr_err;
        merged_arr = new_arr;
        //  index = i;
      }
    }

    //   ROS_INFO_STREAM("Split at "<<index<<" of "<<backwards.size()-1);
    
    return merged_arr;
  }

  double CartesianTrajectories::findMaxTime(const std::vector<KDL::JntArray>& traj) {

    double maxTime = 0;

    for (uint i = 0 ; i < traj.size()-1 ; ++i)
      {
        for (uint j = 0 ; j < joint_velocities.size() ; ++j)
          {
            double vel = joint_velocities[j];
            double v_max = std::max(1e-5, vel);
            double dq1 = traj[i](j);
            double dq2 = traj[i+1](j);
            double t_min = std::abs(dq2-dq1) / v_max;
            
            if (t_min > maxTime)
              maxTime = t_min;
          }
      }
    return maxTime;
  }

                                   
  void CartesianTrajectories::applyVelocityConstraints(const trajectory_msgs::JointTrajectory& newTraj, std::vector<double>& timediff) {   

    timediff.clear();
    timediff.resize(newTraj.points.size(),0);

    for (uint i = 0 ; i < newTraj.points.size()-1 ; ++i)
      {

        trajectory_msgs::JointTrajectoryPoint curr_waypoint = newTraj.points[i];
        trajectory_msgs::JointTrajectoryPoint next_waypoint = newTraj.points[i+1];

        for (uint j = 0 ; j < joint_velocities.size() ; ++j)
          {
            double vel = joint_velocities[j];
            double v_max = std::max(1e-5, vel);
            double dq1 = curr_waypoint.positions[j];
            double dq2 = next_waypoint.positions[j];
            double t_min = std::abs(dq2-dq1) / v_max;

            if (t_min > timediff[i])
              timediff[i] = t_min;
          }
      }

  }
  
  void CartesianTrajectories::updateTrajectory(trajectory_msgs::JointTrajectory& newTraj, const std::vector<double>& timediff) {

    // Error check
    if (timediff.empty())
      return;
    
    double time_sum = 0.0;
    
    newTraj.points[0].time_from_start = ros::Duration(time_sum);

    // Times
    for (int i = 1; i < newTraj.points.size(); ++i) {
      time_sum+=timediff[i-1];
      // Update the time between the waypoints in the robot_trajectory.
      newTraj.points[i].time_from_start = ros::Duration(time_sum);
    }
  }
  
}
