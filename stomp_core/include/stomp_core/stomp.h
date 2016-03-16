/*
 * stomp.h
 *
 *  Created on: Mar 7, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "stomp_core/stomp_core_utils.h"
#include "stomp_core/task.h"
#include "stomp_core/multivariate_gaussian.h"

namespace stomp_core
{

namespace TrajectoryInitializations
{
enum TrajectoryInitialization
{
  LINEAR_INTERPOLATION = 1,
  CUBIC_POLYNOMIAL_INTERPOLATION,
  MININUM_CONTROL_COST
};
}

struct StompConfiguration
{
  // General settings
  int num_iterations;
  int num_iterations_after_valid;   /**< Stomp will stop optimizing this many iterations after finding a valid solution */
  int num_timesteps;
  int num_dimensions;               /** parameter dimensionality */
  double delta_t;               /** time change between consecutive points */
  TrajectoryInitializations::TrajectoryInitialization initialization_method;

  // Noisy trajectory generation
  int num_rollouts_per_iteration; /**< Number of noisy trajectories*/
  int min_rollouts; /**< There be no less than min_rollouts computed on each iteration */
  int max_rollouts; /**< The combined number of new and old rollouts during each iteration shouldn't exceed this value */
  NoiseGeneration noise_generation;

  // Cost calculation
  double control_cost_weight;  /**< Percentage of the trajectory accelerations cost to be applied in the total cost calculation >*/
};


class Stomp
{
public:
  Stomp(const StompConfiguration& config,TaskPtr task);
  virtual ~Stomp();

  bool solve(const std::vector<double>& first,const std::vector<double>& last,
             std::vector<Eigen::VectorXd>& parameters_optimized);
  bool solve(const std::vector<Eigen::VectorXd>& initial_parameters,
             std::vector<Eigen::VectorXd>& parameters_optimized);
  bool cancel();


protected:

  // initialization methods
  bool resetVariables();
  bool computeInitialTrajectory(const std::vector<double>& first,const std::vector<double>& last);

  // optimization methods
  bool runSingleIteration();
  bool generateNoisyRollouts();
  bool filterNoisyRollouts();
  bool computeNoisyRolloutsCosts();
  bool computeRolloutsStateCosts();
  bool computeRolloutsControlCosts();
  bool computeProbabilities();
  bool updateParameters();
  bool filterUpdatedParameters();
  bool computeOptimizedCost();

  // thread safe methods
  void setProceed(bool proceed);
  bool getProceed();

  // noise generation variables
  void updateNoiseStddev();

protected:

  // process control
  boost::mutex proceed_mutex_;
  bool proceed_;
  TaskPtr task_;
  StompConfiguration config_;
  unsigned int current_iteration_;

  // optimized parameters
  bool parameters_valid_;         /**< whether or not the optimized parameters are valid */
  double parameters_total_cost_;  /**< Total cost of the optimized parameters */
  std::vector<Eigen::VectorXd> initial_control_cost_parameters_;    /**< [Dimensions][timesteps]*/
  std::vector<Eigen::VectorXd> parameters_optimized_;               /**< [Dimensions][timesteps]*/
  Eigen::VectorXd parameters_state_costs_;                          /**< [timesteps]*/
  std::vector<Eigen::VectorXd> parameters_control_costs_;           /**< [Dimensions][timesteps]*/
  Eigen::VectorXd temp_parameter_updates_;                          /**< [timesteps]*/

  // noise generation
  std::vector<double> noise_stddevs_;
  boost::shared_ptr<MultivariateGaussian> mv_gaussian_;
  Eigen::VectorXd temp_noise_array_;


  // rollouts
  std::vector<Rollout> noisy_rollouts_;
  std::vector<Rollout> reused_rollouts_;     /**< Used for reordering arrays based on cost */
  int num_active_rollouts_;

  // finite difference and optimization matrices
  Eigen::MatrixXd finite_diff_matrix_A_;            /**< [timesteps x timesteps], Referred to as 'A' in the literature */
  Eigen::MatrixXd control_cost_matrix_R_;           /**< [timesteps x timesteps], Referred to as 'R = A x A_transpose' in the literature */
  Eigen::MatrixXd inv_control_cost_matrix_R_;       /**< [timesteps x timesteps], R^-1 ' matrix */
  Eigen::MatrixXd projection_matrix_M_;             /**< [timesteps x timesteps], Projection smoothing matrix  M > */
  Eigen::MatrixXd inv_projection_matrix_M_;         /**< [timesteps x timesteps], Inverse projection smoothing matrix M-1 >*/


};

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_ */
