/*
 * stomp_optimization_task.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ros-ubuntu
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_

#include <stomp_core/task.h>

namespace stomp_moveit
{

class StompOptimizationTask: public stomp_core::Task
{
public:
  StompOptimizationTask(moveit::core::RobotModelConstPtr robot_model_ptr, std::string group_name);
  virtual ~StompOptimizationTask();

  /**
   * @brief calls the updatePlanningScene method of each cost function and filter class it contains
   * @param planning_scene_ptr a smart pointer to the planning scene
   */
  bool updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene_ptr);

  /**
   * @brief calls each cost function class in order to compute total the state costs
   * as a function of the parameters for each time step.
   *
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param costs vector containing the state costs per timestep.
   * @param iteration_number
   * @param rollout_number index of the noisy trajectory whose cost is being evaluated.
   * @param validity whether or not the trajectory is valid
   * @return true if cost were properly computed
   */
  virtual bool computeCosts(std::vector<Eigen::VectorXd>& parameters,
                       Eigen::VectorXd& costs,
                       const int iteration_number,
                       const int rollout_number,
                       bool& validity) const = 0 ;

  /**
   * Filters the given noisy parameters which is applied after noisy trajectory generation. It could be used for clipping
   * of joint limits or projecting into the null space of the Jacobian.
   *
   * @param parameters
   * @return false if no filtering was done
   */
  virtual bool filterNoisyParameters(std::vector<Eigen::VectorXd>& parameters) const {return false;};

  /**
   * Filters the given parameters which is applied after the update. It could be used for clipping of joint limits
   * or projecting into the null space of the Jacobian.
   *
   * @param parameters
   * @return false if no filtering was done
   */
  virtual bool filterParameters(std::vector<Eigen::VectorXd>& parameters) const {return false;};
};

} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_ */
