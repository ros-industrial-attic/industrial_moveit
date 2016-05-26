/*
 * collision_check.h
 *
 *  Created on: Mar 30, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_COLLISION_CHECK_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_COLLISION_CHECK_H_

#include <Eigen/Sparse>
#include <moveit/robot_model/robot_model.h>
#include <industrial_collision_detection/collision_robot_industrial.h>
#include <industrial_collision_detection/collision_world_industrial.h>
#include "stomp_moveit/cost_functions/stomp_cost_function.h"

namespace stomp_moveit
{
namespace cost_functions
{

class CollisionCheck : public StompCostFunction
{
public:
  CollisionCheck();
  virtual ~CollisionCheck();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;


  /**
   * @brief computes the state costs as a function of the parameters for each time step.
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'   *
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory whose cost is being evaluated.   *
   * @param costs             vector containing the state costs per timestep.
   * @param validity          whether or not the trajectory is valid
   * @return true if cost were properly computed
   */
  virtual bool computeCosts(const Eigen::MatrixXd& parameters,
                            std::size_t start_timestep,
                            std::size_t num_timesteps,
                            int iteration_number,
                            int rollout_number,
                            Eigen::VectorXd& costs,
                            bool& validity) override;

  virtual std::string getGroupName() const override
  {
    return group_name_;
  }


  virtual std::string getName() const override
  {
    return "CollisionCheck/" + group_name_;
  }

  virtual void done(bool success,int total_iterations,double final_cost) override;

protected:

  std::string name_;

  // robot details
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  moveit::core::RobotStatePtr robot_state_;

  // planning context information
  planning_scene::PlanningSceneConstPtr planning_scene_;
  moveit_msgs::MotionPlanRequest plan_request_;

  // parameters
  double collision_penalty_;
  double cost_decay_;

  // cost smoothing
  Eigen::SparseMatrix<double,Eigen::RowMajor> exp_smoothing_matrix_; //square matrix of size = num_timesteps + WINDOW_SIZE -1
  Eigen::VectorXd costs_padded_;
  Eigen::VectorXd intermediate_costs_slots_;                          // logical array of size num_timesteps
  Eigen::VectorXd min_costs_;                                         // stores the minimum costs, size: num_timesteps

  // collision
  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionRobotIndustrialConstPtr collision_robot_;
  collision_detection::CollisionWorldIndustrialConstPtr collision_world_;




};

} /* namespace cost_functions */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_COST_FUNCTIONS_COLLISION_CHECK_H_ */
