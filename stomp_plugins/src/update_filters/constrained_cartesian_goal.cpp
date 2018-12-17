/**
 * @file constrained_cartesian_goal.cpp
 * @brief This defines a underconstrained goal update filter.
 *
 * This will force goal constraints into the task space.
 *
 * @author Jorge Nicho
 * @date June 3, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
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
#include <stomp_plugins/update_filters/constrained_cartesian_goal.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::ConstrainedCartesianGoal,stomp_moveit::update_filters::StompUpdateFilter);

static int CARTESIAN_DOF_SIZE = 6;
static const double DEFAULT_POS_TOLERANCE = 0.001;
static const double DEFAULT_ROT_TOLERANCE = 0.01;

namespace stomp_moveit
{
namespace update_filters
{

ConstrainedCartesianGoal::ConstrainedCartesianGoal():
    name_("ConstrainedCartesianGoal")
{

}

ConstrainedCartesianGoal::~ConstrainedCartesianGoal()
{

}

bool ConstrainedCartesianGoal::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  robot_model_ = robot_model_ptr;
  ik_solver_.reset(new stomp_moveit::utils::kinematics::IKSolver(robot_model_ptr,group_name));

  return configure(config);
}

bool ConstrainedCartesianGoal::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;
  return true;
}

bool ConstrainedCartesianGoal::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace utils::kinematics;

  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  int num_joints = joint_group->getActiveJointModels().size();
  tool_link_ = joint_group->getLinkModelNames().back();
  state_.reset(new RobotState(robot_model_));
  robotStateMsgToRobotState(req.start_state,*state_);

  // update kinematic model
  ik_solver_->setKinematicState(*state_);

  const std::vector<moveit_msgs::Constraints>& goals = req.goal_constraints;
  if(goals.empty())
  {
    ROS_ERROR("A goal constraint was not provided");
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // save tool goal pose and constraints
  bool found_goal = false;
  for(const auto& g: goals)
  {
    if(utils::kinematics::isCartesianConstraints(g))
    {
      // tool cartesian goal data
      state_->updateLinkTransforms();
      Eigen::Affine3d start_tool_pose = state_->getGlobalLinkTransform(tool_link_);
      boost::optional<moveit_msgs::Constraints> cartesian_constraints = utils::kinematics::curateCartesianConstraints(g,start_tool_pose);
      if(cartesian_constraints.is_initialized())
      {
        found_goal = utils::kinematics::decodeCartesianConstraint(robot_model_,cartesian_constraints.get(),tool_goal_pose_,tool_goal_tolerance_,
                                                                  robot_model_->getRootLinkName());
      }

      if(found_goal)
      {
        break;  // a Cartesian goal was found, done
      }
    }
  }


  // compute the tool goal pose from the goal joint configuration if there exists any
  if(!found_goal)
  {
    ROS_DEBUG("%s a cartesian goal pose in MotionPlanRequest was not provided,calculating it from FK",getName().c_str());

    for(const auto& g: goals)
    {
      // check joint constraints
      if(g.joint_constraints.empty())
      {
        ROS_WARN_STREAM("No joint values for the goal were found");
        continue;
      }

      // compute FK to obtain tool pose
      const std::vector<moveit_msgs::JointConstraint>& joint_constraints = g.joint_constraints;

      // copying goal values into state
      for(auto& jc: joint_constraints)
      {
        state_->setVariablePosition(jc.joint_name,jc.position);
      }

      // storing tool goal pose and tolerance
      state_->update(true);
      tool_goal_pose_ = state_->getGlobalLinkTransform(tool_link_);
      tool_goal_tolerance_.resize(CARTESIAN_DOF_SIZE);
      double ptol = DEFAULT_POS_TOLERANCE;
      double rtol = DEFAULT_ROT_TOLERANCE;
      tool_goal_tolerance_ << ptol, ptol, ptol, rtol, rtol, rtol;
      found_goal = true;
      break;
    }
  }

  if(!found_goal)
  {
    ROS_ERROR("%s No valid goal pose was found",getName().c_str());
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  error_code.val = error_code.SUCCESS;
  return true;
}

bool ConstrainedCartesianGoal::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    const Eigen::MatrixXd& parameters,
                    Eigen::MatrixXd& updates,
                    bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;
  using namespace stomp_moveit::utils;


  filtered = false;
  VectorXd goal_joint_pose;
  VectorXd seed = parameters.rightCols(1) + updates.rightCols(1);

  // solve kinematics
  if(ik_solver_->solve(seed,tool_goal_pose_,goal_joint_pose,tool_goal_tolerance_))
  {
    filtered = true;
    updates.rightCols(1) = goal_joint_pose - parameters.rightCols(1);
  }
  else
  {
    ROS_DEBUG("%s failed failed to solve ik using noisy goal pose as seed, zeroing updates",getName().c_str());
    filtered = true;
    updates.rightCols(1) = Eigen::VectorXd::Zero(updates.rows());
  }

  return true;
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
