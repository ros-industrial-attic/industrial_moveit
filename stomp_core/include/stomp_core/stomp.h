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
  int num_iterations;
  int num_iterations_after_valid;   /**< Stomp will stop optimizing this many iterations after finding a valid solution */
  int num_timesteps;
  int dimensions;               /** parameter dimensionality */
  double delta_t;               /** time change between consecutive points */
  TrajectoryInitializations::TrajectoryInitialization initialization_method;

  // Noisy trajectory generation
  int num_rollouts; /**< Number of noisy trajectories*/
  int min_rollouts; /**< There be no less than min_rollouts computed on each iteration */
  int max_rollouts; /**< The combined number of new and old rollouts during each iteration shouldn't exceed this value */
  NoiseGeneration noise_generation;
};


class Stomp
{
public:
  Stomp(const StompConfiguration& config,TaskPtr task);
  virtual ~Stomp();

  bool solve(const std::vector<double>& first,const std::vector<double>& last,
             std::vector<Eigen::VectorXd>& optimized_parameters);
  bool solve(const std::vector<Eigen::VectorXd>& initial_parameters,
             std::vector<Eigen::VectorXd>& optimized_parameters);
  bool cancel();


protected:

  // optimization algorithm steps
  bool initializeOptimizationVariables();
  bool computeInitialTrajectory(const std::vector<double>& first,const std::vector<double>& last);

  bool runOptimization();
  bool runSingleIteration();
  bool generateNoisyRollouts();
  bool computeRolloutsCosts();
  bool computeProbabilities();
  bool updateParameters();
  bool computeOptimizedCost();

  // thread safe methods
  void setProceed(bool proceed);
  bool getProceed();

  // noise generation variables update
  void updateNoiseStddev();

protected:

  // process control
  boost::mutex proceed_mutex_;
  bool proceed_;
  TaskPtr task_;
  StompConfiguration config_;
  unsigned int current_iteration_;

  // optimized parameters
  bool optimized_parameters_valid_;         /**< whether or not the optimized parameters are valid */
  double optimized_parameters_total_cost_;  /**< Total cost of the optimized parameters */

  // noise generation
  std::vector<double> noise_stddevs_;
  boost::shared_ptr<MultivariateGaussian> mv_gaussian_;


  // rollouts
  std::vector<Rollout> noisy_rollouts_;
  std::vector<Rollout> reused_rollouts_;     /**< Used for reordering arrays based on cost */
  int num_active_rollouts_;

  // finite difference and optimization matrices
  Eigen::MatrixXd acc_diff_matrix_;         /**< [timesteps x timesteps], Referred to as 'A' in the literature */
  Eigen::MatrixXd control_cost_matrix_;     /**< [timesteps x timesteps], Referred to as 'R = A x A_transpose' in the literature */
  Eigen::MatrixXd smooth_update_matrix_;    /**< [timesteps x timesteps], Smoothing 'M = R^-1 ' matrix */

  std::vector<Eigen::VectorXd> initial_control_cost_parameters_;    /**< [Dimensions][timesteps]*/
  std::vector<Eigen::VectorXd> optimized_parameters_;               /**< [Dimensions][timesteps]*/


};

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_ */
