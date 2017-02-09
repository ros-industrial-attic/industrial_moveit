/**
 * @file constrained_ik_plugin.cpp
 * @brief Constrained inverse kinematic plugin for moveit.
 *
 * @author dsolomon
 * @date Sep 15, 2013
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "constrained_ik/moveit_interface/constrained_ik_plugin.h"

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(constrained_ik::ConstrainedIKPlugin, kinematics::KinematicsBase)

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace constrained_ik
{

ConstrainedIKPlugin::ConstrainedIKPlugin():active_(false), dimension_(0)
{
}

bool ConstrainedIKPlugin::isActive() const
{
  if(active_)
    return true;
  ROS_ERROR("kinematics not active");
  return false;
}

bool ConstrainedIKPlugin::initialize(const std::string& robot_description,
                                     const std::string& group_name,
                                     const std::string& base_name,
                                     const std::string& tip_name,
                                     double search_discretization)
{
  setValues(robot_description, group_name, base_name, tip_name, search_discretization);

  // init robot model
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_STREAM("URDF and SRDF must be loaded for Constrained Ik solver to work.");
    return false;
  }

  // instatiating a robot model
  robot_model_ptr_.reset(new robot_model::RobotModel(urdf_model,srdf));
  if(!robot_model_ptr_)
  {
    ROS_ERROR_STREAM("Could not load URDF model from " << robot_description);
    active_ = false;
    return false;
  }

  const robot_model::JointModelGroup* joint_model_group = robot_model_ptr_->getJointModelGroup(group_name);
  if (!joint_model_group)
    return false;

  if (!joint_model_group->isChain())
  {
    ROS_ERROR_NAMED("clik", "Group '%s' is not a chain.", group_name.c_str());
    return false;
  }

  robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));

  // initializing planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_ptr_));

  //initialize kinematic solver with robot info
  if (!kin_.init(joint_model_group))
  {
    ROS_ERROR("Could not initialize BasicIK");
    active_ = false;
  }
  else
  {
    dimension_ = kin_.numJoints();
    kin_.getJointNames(joint_names_);
    kin_.getLinkNames(link_names_);
    active_ = true;
  }

  try
  {
    solver_.reset(new Constrained_IK());
    std::string constraint_param = "constrained_ik_solver/" + group_name + "/constraints";
    solver_->addConstraintsFromParamServer(constraint_param);
    solver_->init(kin_); // inside try because it has the potential to throw and error
  }
  catch (exception &e)
  {
    ROS_ERROR_STREAM("Caught exception from solver_: " << e.what());
    return false;
  }

  return active_;
}

bool ConstrainedIKPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                        const std::vector<double> &ik_seed_state,
                                        std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const
{
  //check that solver is ready and properly configured
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  if(ik_seed_state.size() != dimension_)
  {
    ROS_ERROR("ik_seed_state does not have same dimension as solver");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  //convert input parameters to required types
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);
  Eigen::Affine3d goal;
  tf::transformKDLToEigen(pose_desired, goal);

  Eigen::VectorXd seed(dimension_), joint_angles(dimension_);
  for(size_t ii=0; ii < dimension_; ++ii)
  {
    seed(ii) = ik_seed_state[ii];
  }

  //Do IK and report results
  try
  {
    if(!solver_->calcInvKin(goal, seed, planning_scene_, joint_angles))
    {
      ROS_ERROR_STREAM("Unable to find IK solution.");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }
  }
  catch (exception &e)
  {
    ROS_ERROR_STREAM("Caught exception from IK: " << e.what());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  solution.resize(dimension_);
  for(size_t ii=0; ii < dimension_; ++ii)
  {
    solution[ii] = joint_angles(ii);
  }

  return true;

}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            std::vector<double> &solution,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    static IKCallbackFn solution_callback = 0;
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            const std::vector<double> &consistency_limits,
                                            std::vector<double> &solution,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    static IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            std::vector<double> &solution,
                                            const IKCallbackFn &solution_callback,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            const std::vector<double> &consistency_limits,
                                            std::vector<double> &solution,
                                            const IKCallbackFn &solution_callback,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{

  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.FAILURE;
    return false;
  }

  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);
  Eigen::Affine3d goal;
  tf::transformKDLToEigen(pose_desired, goal);

  if(dimension_ != ik_seed_state.size()){
    ROS_ERROR("dimension_ and ik_seed_state are of different sizes");
    return(false);
  }
  Eigen::VectorXd seed(dimension_), joint_angles;
  for(size_t ii=0; ii < dimension_; ii++)
  {
    seed(ii) = ik_seed_state[ii];
  }

  //Do the IK
  bool success(true);
  try
  {
    if(!solver_->calcInvKin(goal, seed, planning_scene_, joint_angles))
    {
      ROS_ERROR_STREAM("Unable to find IK solution.");
      error_code.val = error_code.NO_IK_SOLUTION;
      success &= false;
    }
  }
  catch (exception &e)
  {
    ROS_ERROR_STREAM("Caught exception from IK: " << e.what());
    error_code.val = error_code.NO_IK_SOLUTION;
    success &= false;
  }

  solution.resize(dimension_);
  for(size_t ii=0; ii < dimension_; ++ii)
  {
    solution[ii] = joint_angles(ii);
  }

  // If there is a solution callback registered, check before returning
  if (solution_callback)
  {
    solution_callback(ik_pose, solution, error_code);
    if(error_code.val != error_code.SUCCESS)
      success &= false;
  }
  // Default: return successfully
  if (success)
    error_code.val = error_code.SUCCESS;

  return success;

}

bool ConstrainedIKPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                        const std::vector<double> &joint_angles,
                                        std::vector<geometry_msgs::Pose> &poses) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  Eigen::VectorXd jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  poses.resize(link_names.size());

  std::vector<KDL::Frame> kdl_poses;
  bool valid = kin_.linkTransforms(jnt_pos_in, kdl_poses, link_names);
  for(size_t ii=0; ii < kdl_poses.size(); ++ii)
  {
    tf::poseKDLToMsg(kdl_poses[ii],poses[ii]);
  }
  //TODO remove this printing
  ROS_INFO_STREAM("poses: ");
  for (size_t ii=0; ii<poses.size(); ++ii)
  {
    ROS_INFO_STREAM(poses[ii]);
  }
  return valid;
}

const std::vector<std::string>& ConstrainedIKPlugin::getJointNames() const
{
  isActive();
  return joint_names_;
}

const std::vector<std::string>& ConstrainedIKPlugin::getLinkNames() const
{
  isActive();
  return link_names_;
}

} //namespace constrained_ik_plugin
