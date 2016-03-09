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
#include "stomp_core/stomp_utils.h"
#include "stomp_core/task.h"

namespace stomp_core
{

struct StompParameters
{
  int max_iterations;
  int timesteps;
  int dimensions;     /** parameter dimensionality */

  // Noisy trajectory generation
  int num_rollouts; /**< Number of noisy trajectories*/
  int min_rollouts; /**< There be no less than min_rollouts computed on each iteration */
  int max_rollouts; /**< The combined number of new and old rollouts during each iteration shouldn't exceed this value */
  NoiseGenerationParams noise_coeffs;
};


class Stomp
{
public:
  Stomp();
  virtual ~Stomp();

  bool solve(const StompParameters& params,TaskPtr task,Eigen::MatrixXf& optimized_parameters);
  bool cancel();


protected:

  // optimization algorithm steps
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

protected:

  boost::mutex proceed_mutex_;
  bool proceed_;
  TaskPtr task_;
  StompParameters optimization_params_;

  // optimized parameters
  bool optimized_parameters_valid_;
  double optimized_parameters_costs_;

};

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_H_ */
