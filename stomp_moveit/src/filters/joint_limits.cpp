/*
 * joint_limits.cpp
 *
 *  Created on: Apr 1, 2016
 *      Author: Jorge Nicho
 */

#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>
#include "stomp_moveit/filters/joint_limits.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::filters::JointLimits,stomp_moveit::filters::StompFilter);

namespace stomp_moveit
{
namespace filters
{

JointLimits::JointLimits():
    lock_start_(true),
    lock_goal_(true)
{
  // TODO Auto-generated constructor stub

}

JointLimits::~JointLimits()
{
  // TODO Auto-generated destructor stub
}

bool JointLimits::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  using namespace moveit::core;

  robot_model_ = robot_model_ptr;
  group_name_ = group_name;

  // creating states
  start_state_.reset(new RobotState(robot_model_));
  goal_state_.reset(new RobotState(robot_model_));

  return configure(config);
}

bool JointLimits::configure(const XmlRpc::XmlRpcValue& config)
{

  // check parameter presence
  auto members = {"lock_start","lock_goal"};
  for(auto& m : members)
  {
    if(!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters",getName().c_str());
      return false;
    }
  }

  try
  {
    XmlRpc::XmlRpcValue c = config;
    lock_start_ = static_cast<bool>(c["lock_start"]);
    lock_goal_ = static_cast<bool>(c["lock_goal"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("JointLimits plugin failed to load parameters %s",e.getMessage().c_str());
    return false;
  }

  return true;
}

bool JointLimits::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  error_code.val = error_code.val | moveit_msgs::MoveItErrorCodes::SUCCESS;

  // saving start state
  if(!robotStateMsgToRobotState(req.start_state,*start_state_))
  {
    ROS_ERROR_STREAM("Failed to save start state");
    return false;
  }
  start_state_->enforceBounds(robot_model_->getJointModelGroup(group_name_));


  // saving goal state
  if(lock_goal_)
  {
    bool goal_state_saved = false;
    for(auto& gc: req.goal_constraints)
    {
      for(auto& jc : gc.joint_constraints)
      {
        goal_state_->setVariablePosition(jc.joint_name,jc.position);
        goal_state_saved = true;
      }
      goal_state_->enforceBounds(robot_model_->getJointModelGroup(group_name_));
      break;
    }

    if(!goal_state_saved)
    {
      ROS_ERROR_STREAM("Failed to save goal state");
      return false;
    }
  }

  return true;
}

bool JointLimits::filter(std::size_t start_timestep,std::size_t num_timesteps,
                         int iteration_number,int rollout_number,Eigen::MatrixXd& parameters,bool& filtered)
{
  using namespace moveit::core;

  filtered = false;
  const JointModelGroup* joint_group  = robot_model_->getJointModelGroup(group_name_);
  const std::vector<const JointModel*>& joint_models = joint_group->getActiveJointModels();
  std::size_t num_joints = joint_group->getActiveJointModelNames().size();
  if(parameters.rows() != num_joints)
  {
    ROS_ERROR("Incorrect number of joints in the 'parameters' matrix");
    return false;
  }

  if(lock_start_)
  {
    for(auto j = 0u; j < num_joints; j++)
    {
      parameters(j,0) =  *start_state_->getJointPositions(joint_models[j]);
    }
  }

  if(lock_goal_)
  {
    auto last_index = parameters.cols()-1;
    for(auto j = 0u; j < num_joints; j++)
    {
      parameters(j,last_index) =  *goal_state_->getJointPositions(joint_models[j]);
    }
  }

  double val;
  for (auto j = 0u; j < num_joints; ++j)
  {
    for (auto t=0u; t< parameters.cols(); ++t)
    {
      val = parameters(j,t);
      if(joint_models[j]->enforcePositionBounds(&val))
      {
        parameters(j,t) = val;
        filtered = true;
      }
    }
  }

  return true;
}

} /* namespace filters */
} /* namespace stomp_moveit */
