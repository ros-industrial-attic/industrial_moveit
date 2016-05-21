/*
 * UNDERCONSTRAINED_GOAL.h
 *
 *  Created on: Apr 21, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_SRC_FILTERS_UNDERCONSTRAINED_GOAL_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_SRC_FILTERS_UNDERCONSTRAINED_GOAL_H_

#include <moveit/robot_model/robot_model.h>
#include <stomp_moveit/filters/stomp_filter.h>
#include <array>
#include <Eigen/Core>

namespace stomp_moveit
{
namespace filters
{

class UnderconstrainedGoal : public StompFilter
{
public:
  UnderconstrainedGoal();
  virtual ~UnderconstrainedGoal();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config);

  virtual bool configure(const XmlRpc::XmlRpcValue& config);

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  /**
   * @brief filters the parameters and modifies the original values
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory whose cost is being evaluated.
   * @param parameters [num_dimensions] x [num_timesteps]
   * @return false if no filtering was applied
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      int rollout_number,
                      Eigen::MatrixXd& parameters,
                      bool& filtered);

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   */
  virtual void done(bool success,int total_iterations,double final_cost){}


  virtual std::string getName() const
  {
    return name_ + "/" + group_name_;
  }

  virtual std::string getGroupName() const
  {
    return group_name_;
  }

protected:

  void reduceJacobian(const Eigen::MatrixXd& jacb,Eigen::MatrixXd& jacb_reduced);
  void calculatePseudoInverse(const Eigen::MatrixXd& jacb,Eigen::MatrixXd& jacb_pseudo_inv);
  void computeTwist(const Eigen::Affine3d& start_pose,
                   const Eigen::Affine3d& end_pose,
                   std::array<bool,6>& nullity,
                   Eigen::VectorXd& twist);

protected:

  std::string name_;
  std::string group_name_;

  // tool goal
  Eigen::Affine3d tool_goal_pose_;

  // underconstrained info
  std::array<bool,6> dof_nullity_;
  std::array<double,6> dof_convergence_thresholds_;
  int num_constrained_dof_;

  // robot
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr state_;
  std::string tool_link_;




};

} /* namespace filters */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_SRC_FILTERS_UNDERCONSTRAINED_GOAL_H_ */
