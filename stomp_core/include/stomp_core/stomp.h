/**
 * @file stomp.h
 * @brief This contains the stomp core algorithm
 *
 * @author Jorge Nicho
 * @date March 7, 2016
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
#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_

#include <atomic>
#include <stomp_core/utils.h>
#include <XmlRpc.h>
#include "stomp_core/task.h"

namespace stomp_core
{

/** @brief The Stomp class */
class Stomp
{
public:
  /**
   * @brief Stomp Constructor
   * @param config Stomp configuration parameters
   * @param task The item to be optimized.
   */
  Stomp(const StompConfiguration& config,TaskPtr task);

  /**
   * @brief Find the optimal solution provided a start and end goal.
   * @param first Start state for the task
   * @param last Final state for the task
   * @param parameters_optimized Optimized solution [parameters][timesteps]
   * @return True if solution was found, otherwise false.
   */
  bool solve(const std::vector<double>& first,const std::vector<double>& last,
             Eigen::MatrixXd& parameters_optimized);

  /**
   * @brief Find the optimal solution provided a start and end goal.
   * @param first Start state for the task
   * @param last Final state for the task
   * @param parameters_optimized Optimized solution [Parameters][timesteps]
   * @return True if solution was found, otherwise false.
   */
  bool solve(const Eigen::VectorXd& first,const Eigen::VectorXd& last,
             Eigen::MatrixXd& parameters_optimized);

  /**
   * @brief Find the optimal solution provided an intial guess.
   * @param initial_parameters A matrix [Parameters][timesteps]
   * @param parameters_optimized The optimized solution [Parameters][timesteps]
   * @return True if solution was found, otherwise false.
   */
  bool solve(const Eigen::MatrixXd& initial_parameters,
             Eigen::MatrixXd& parameters_optimized);

  /**
   * @brief Sets the configuration and resets all internal variables
   * @param config Stomp Configuration struct
   */
  void setConfig(const StompConfiguration& config);

  /**
   * @brief Cancel optimization in progress. (Thread-Safe)
   * This method is thead-safe.
   * @return True if sucessful, otherwise false.
   */
  bool cancel();

  /**
   * @brief Resets all internal variables
   * @return True if sucessful, otherwise false.
   */
  bool clear();


protected:

  // initialization methods
  /**
   * @brief Reset all internal variables.
   * @return True if sucessful, otherwise false.
   */
  bool resetVariables();

  /**
   * @brief Computes an inital guess at a solution given a start and end state
   * @param first Start state for the task
   * @param last Final state for the task
   * @return True if sucessful, otherwise false.
   */
  bool computeInitialTrajectory(const std::vector<double>& first,const std::vector<double>& last);

  // optimization steps
  /**
   * @brief Run a single iteration of the stomp algorithm
   * @return True if it was able to succesfully perform a single iteration. False
   * is returned when an error is encounter at one of the many optimization steps.
   */
  bool runSingleIteration();

  /**
   * @brief Generate a set of noisy rollouts
   * @return True if sucessful, otherwise false.
   */
  bool generateNoisyRollouts();

  /**
   * @brief Applies the optimization task's filter methods to the noisy trajectories.
   * @return True if sucessful, otherwise false.
   */
  bool filterNoisyRollouts();

  /**
   * @brief Computes the total cost for each of the noisy rollouts.
   * @return True if sucessful, otherwise false.
   */
  bool computeNoisyRolloutsCosts();

  /**
   * @brief Computes the cost at every timestep for each noisy rollout.
   * @return True if sucessful, otherwise false.
   */
  bool computeRolloutsStateCosts();

  /**
   * @brief Compute the control cost for each noisy rollout.
   * This is the sum of the acceleration squared, then each
   * noisy rollouts control cost is divided by the maximum
   * control cost found amoung the noisy rollouts.
   * @return True if sucessful, otherwise false.
   */
  bool computeRolloutsControlCosts();

  /**
   * @brief Computes the probability from the state cost at every timestep for each noisy rollout.
   * @return True if sucessful, otherwise false.
   */
  bool computeProbabilities();

  /**
   * @brief Computes update from probabilities using convex combination
   * @return True if sucessful, otherwise false.
   */
  bool updateParameters();

  /**
   * @brief Computes the optimized trajectory cost [Control Cost + State Cost]
   * If the current cost is not less than the previous cost the
   * parameters are reset to the previous iteration's parameters.
   * @return
   */
  bool computeOptimizedCost();

protected:

  // process control
  std::atomic<bool> proceed_;                      /**< @brief Used to determine if the optimization has been cancelled. */
  TaskPtr task_;                                   /**< @brief The task to be optimized. */
  StompConfiguration config_;                      /**< @brief Configuration parameters. */
  unsigned int current_iteration_;                 /**< @brief Current iteration for the optimization. */

  // optimized parameters
  bool parameters_valid_;                          /**< @brief whether or not the optimized parameters are valid */
  bool parameters_valid_prev_;                     /**< @brief whether or not the optimized parameters from the previous iteration are valid */
  double parameters_total_cost_;                   /**< @brief Total cost of the optimized parameters */
  double current_lowest_cost_;                     /**< @brief Hold the lowest cost of the optimized parameters */
  Eigen::MatrixXd parameters_optimized_;           /**< @brief A matrix [dimensions][timesteps] of the optimized parameters. */

  Eigen::MatrixXd parameters_updates_;             /**< @brief A matrix [dimensions][timesteps] of the parameter updates*/
  Eigen::VectorXd parameters_state_costs_;         /**< @brief A vector [timesteps] of the parameters state costs */
  Eigen::MatrixXd parameters_control_costs_;       /**< @brief A matrix [dimensions][timesteps] of the parameters control costs*/

  // rollouts
  std::vector<Rollout> noisy_rollouts_;            /**< @brief Holds the noisy rollouts */
  std::vector<Rollout> reused_rollouts_;           /**< @brief Used for reordering arrays based on cost */
  int num_active_rollouts_;                        /**< @brief Number of active rollouts */

  // finite difference and optimization matrices
  int num_timesteps_padded_;                       /**< @brief The number of timesteps to pad the optimization with: timesteps + 2*(FINITE_DIFF_RULE_LENGTH - 1) */
  int start_index_padded_;                         /**< @brief The index corresponding to the start of the non-paded section in the padded arrays */
  Eigen::MatrixXd finite_diff_matrix_A_padded_;    /**< @brief The finite difference matrix including padding */
  Eigen::MatrixXd control_cost_matrix_R_padded_;   /**< @brief The control cost matrix including padding */
  Eigen::MatrixXd control_cost_matrix_R_;          /**< @brief A matrix [timesteps][timesteps], Referred to as 'R = A x A_transpose' in the literature */
  Eigen::MatrixXd inv_control_cost_matrix_R_;      /**< @brief A matrix [timesteps][timesteps], R^-1 ' matrix */


};

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_ */
