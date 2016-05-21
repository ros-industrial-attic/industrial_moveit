/*
 * goal_orientation_constraint.cpp
 *
 *  Created on: Apr 21, 2016
 *      Author: Jorge Nicho
 */

#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <stomp_moveit/filters/underconstrained_goal.h>

namespace stomp_moveit
{
namespace filters
{

UnderconstrainedGoal::UnderconstrainedGoal()
{
  // TODO Auto-generated constructor stub

}

UnderconstrainedGoal::~UnderconstrainedGoal()
{
  // TODO Auto-generated destructor stub
}

bool UnderconstrainedGoal::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  return true;
}

bool UnderconstrainedGoal::configure(const XmlRpc::XmlRpcValue& config)
{
  return true;
}

bool UnderconstrainedGoal::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace Eigen;
  using namespace moveit::core;


  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  tool_link_ = joint_group->getLinkModelNames().back();
  state_.reset(new RobotState(robot_model_));
  robotStateMsgToRobotState(req.start_state,*state_);

  const std::vector<moveit_msgs::Constraints>& goals = req.goal_constraints;
  if(goals.empty())
  {
    ROS_ERROR("A goal constraint was not provided");
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // storing tool goal pose
  if(goals.front().position_constraints.empty() ||
      goals.front().orientation_constraints.empty())
  {
    ROS_WARN("A goal constraint for the tool link was not provided, using forward kinematics");

    // check joint constraints
    if(goals.front().joint_constraints.empty())
    {
      ROS_ERROR_STREAM("No joint values for the goal were found");
      error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    // compute FK to obtain tool pose
    const std::vector<moveit_msgs::JointConstraint>& joint_constraints = goals.front().joint_constraints;

    // copying goal values into state
    for(auto& jc: joint_constraints)
    {
      state_->setVariablePosition(jc.joint_name,jc.position);
    }

    state_->update(true);
    state_->enforceBounds(joint_group);
    tool_goal_pose_ = state_->getGlobalLinkTransform(tool_link_);
  }
  else
  {
    // tool goal
    const moveit_msgs::PositionConstraint& pos_constraint = goals.front().position_constraints.front();
    const moveit_msgs::OrientationConstraint& orient_constraint = goals.front().orientation_constraints.front();

    geometry_msgs::Pose pose;
    pose.position = pos_constraint.constraint_region.primitive_poses[0].position;
    pose.orientation = orient_constraint.orientation;
    tf::poseMsgToEigen(pose,tool_goal_pose_);
  }

  error_code.val = error_code.SUCCESS;
  return true;
}


bool UnderconstrainedGoal::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    int rollout_number,
                    Eigen::MatrixXd& parameters,
                    bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;

  VectorXd joint_pose = parameters.rightCols(1);
  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  state_->setJointGroupPositions(joint_group,joint_pose);
  Affine3d tool_current_pose = state_->getGlobalLinkTransform(tool_link_);

  // computing twist vector
  VectorXd tool_twist;
  computeTwist(tool_current_pose,tool_goal_pose_,dof_nullity_,tool_twist);

  return true;
}

void UnderconstrainedGoal::reduceJacobian(const Eigen::MatrixXd& jacb,Eigen::MatrixXd& jacb_reduced)
{

}

void UnderconstrainedGoal::calculatePseudoInverse(const Eigen::MatrixXd& jacb,Eigen::MatrixXd& jacb_pseudo_inv)
{

}


void UnderconstrainedGoal::computeTwist(const Eigen::Affine3d& p0,
                                        const Eigen::Affine3d& pf,
                                        std::array<bool,6>& nullity,Eigen::VectorXd& twist)
{
  twist = Eigen::VectorXd::Zero(nullity.size());
  Eigen::Vector3d twist_pos = pf.translation() - p0.translation();

  // relative rotation -> R = inverse(R0) * Rf
  Eigen::AngleAxisd relative_rot(p0.rotation().transpose() * pf.rotation());
  double angle = relative_rot.angle();
  Eigen::Vector3d axis = relative_rot.axis().normalized();

  // forcing angle to range [-pi , pi]
  while( (angle > M_PI) || (angle < -M_PI))
  {
    angle = (angle >  M_PI) ? (angle - 2*M_PI) : angle;
    angle = (angle < -M_PI )? (angle + 2*M_PI) : angle;
  }

  // creating twist rotation relative to world
  Eigen::Vector3d twist_rot = p0.rotation() * axis * angle;

  // assigning into full twist vector
  twist.head(3) = twist_pos;
  twist.tail(3) = twist_rot;

  // nullifying underconstrained
  for(auto i = 0u; i < nullity.size(); i++)
  {
    if(!nullity[i])
    {
      twist(i) = 0;
    }
  }


}

} /* namespace filters */
} /* namespace stomp_moveit */
