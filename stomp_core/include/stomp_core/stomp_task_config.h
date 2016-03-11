/*
 * stomp_task_config.h
 *
 *  Created on: Mar 8, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_TASK_CONFIG_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_TASK_CONFIG_H_

#include <moveit_msgs/MotionPlanRequest.h>
#include "stomp_core/stomp_cost_function.h"

namespace stomp_core
{

struct StompTaskConfig
{

  std::string group_name;
  int num_iterations;
  int timesteps;
  float velocity_scaling_factor;    /**< used for scaling the max joint velocities.  The scaled velocities will determine the
                                         planned trajectory duration*/
  float max_delta_t;                /**< Maximum time step between consecutive points */

  // Noisy trajectory generation
  int num_rollouts; /**< Number of noisy trajectories*/
  int min_rollouts; /**< There be no less than min_rollouts computed on each iteration */
  int max_rollouts; /**< The combined number of new and old rollouts during each iteration shouldn't exceed this value */
  NoiseGenerationConfig noise_coeffs;

  // Trajectory cost function
  std::vector<StompCostFunctionPtr> cost_functions;

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_TASK_CONFIG_H_ */
