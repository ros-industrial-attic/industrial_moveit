/**
 * @file simple_optimization_task.h
 * @brief A simple task for showing how to use STOMP
 *
 * @author Jorge Nicho
 * @date Dec 14, 2016
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

#ifndef EXAMPLES_SIMPLE_OPTIMIZATION_TASK_H_
#define EXAMPLES_SIMPLE_OPTIMIZATION_TASK_H_

#include <stomp_core/task.h>

namespace stomp_core_examples
{

//! [SimpleOptimizationTask Inherit]
/** @brief A dummy task for testing STOMP */
class SimpleOptimizationTask: public stomp_core::Task
{
public:
  /**
   * @brief A simple task for demonstrating how to use Stomp
   * @param parameters_bias default parameter bias used for computing cost.
   * @param bias_thresholds threshold to determine whether two trajectories are equal
   * @param std_dev standard deviation used for generating noisy parameters
   */
  SimpleOptimizationTask(const Eigen::MatrixXd& parameters_bias,
            const std::vector<double>& bias_thresholds,
            const std::vector<double>& std_dev):
              parameters_bias_(parameters_bias),
              bias_thresholds_(bias_thresholds),
              std_dev_(std_dev)
  {

    // generate smoothing matrix
    int num_timesteps = parameters_bias.cols();
    stomp_core::generateSmoothingMatrix(num_timesteps,1.0,smoothing_M_);
    srand(time(0));

  }

//! [SimpleOptimizationTask Inherit]
  /**
   * @brief Generates a noisy trajectory from the parameters.
   * @param parameters        A matrix [num_dimensions][num_parameters] of the current optimized parameters
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The index of the noisy trajectory.
   * @param parameters_noise  The parameters + noise
   * @param noise             The noise applied to the parameters
   * @return True if cost were properly computed, otherwise false
   */
  bool generateNoisyParameters(const Eigen::MatrixXd& parameters,
                               std::size_t start_timestep,
                               std::size_t num_timesteps,
                               int iteration_number,
                               int rollout_number,
                               Eigen::MatrixXd& parameters_noise,
                               Eigen::MatrixXd& noise) override
  {
    double rand_noise;
    for(std::size_t d = 0; d < parameters.rows(); d++)
    {
      for(std::size_t t = 0; t < parameters.cols(); t++)
      {
        rand_noise = static_cast<double>(rand()%RAND_MAX)/static_cast<double>(RAND_MAX - 1); // 0 to 1
        rand_noise = 2*(0.5 - rand_noise);
        noise(d,t) =  rand_noise*std_dev_[d];
      }
    }

    parameters_noise = parameters + noise;

    return true;
  }

  /**
   * @brief computes the state costs as a function of the distance from the bias parameters
   * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param costs             A vector containing the state costs per timestep.
   * @param validity          Whether or not the trajectory is valid
   * @return True if cost were properly computed, otherwise false
   */
  bool computeCosts(const Eigen::MatrixXd& parameters,
                    std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    Eigen::VectorXd& costs,
                    bool& validity) override
  {
    return computeNoisyCosts(parameters,start_timestep,num_timesteps,iteration_number,-1,costs,validity);
  }

  /**
   * @brief computes the state costs as a function of the distance from the bias parameters
   * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The index of the noisy trajectory.
   * @param costs             A vector containing the state costs per timestep.
   * @param validity          Whether or not the trajectory is valid
   * @return True if cost were properly computed, otherwise false
   */
  bool computeNoisyCosts(const Eigen::MatrixXd& parameters,
                         std::size_t start_timestep,
                         std::size_t num_timesteps,
                         int iteration_number,
                         int rollout_number,
                         Eigen::VectorXd& costs,
                         bool& validity) override
  {
    costs.setZero(num_timesteps);
    double diff;
    double cost = 0.0;
    validity = true;

    for(std::size_t t = 0u; t < num_timesteps; t++)
    {
      cost = 0;
      for(std::size_t d = 0u; d < parameters.rows() ; d++)
      {

        diff = std::abs(parameters(d,t) - parameters_bias_(d,t));
        if( diff > std::abs(bias_thresholds_[d]))
        {
          cost += diff;
          validity = false;
        }
      }

      costs(t) = cost;
    }

    return true;
  }

  /**
   * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint limits
   * or projecting into the null space of the Jacobian.
   *
   * @param start_timestep    The start index into the 'parameters' array, usually 0.
   * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The optimized parameters
   * @param updates           The updates to the parameters
   * @return                  True if successful, otherwise false
   */
  bool filterParameterUpdates(std::size_t start_timestep,
                              std::size_t num_timesteps,
                              int iteration_number,
                              const Eigen::MatrixXd& parameters,
                              Eigen::MatrixXd& updates) override
  {
    return smoothParameterUpdates(start_timestep,num_timesteps,iteration_number,updates);
  }


protected:

  /**
   * @brief Perform a smooth update given a noisy update
   * @param start_timestep starting timestep
   * @param num_timesteps number of timesteps
   * @param iteration_number number of interations allowed
   * @param updates returned smooth update
   * @return True if successful, otherwise false
   */
  bool smoothParameterUpdates(std::size_t start_timestep,
                              std::size_t num_timesteps,
                              int iteration_number,
                              Eigen::MatrixXd& updates)
  {

    for(auto d = 0u; d < updates.rows(); d++)
    {
      updates.row(d).transpose() = smoothing_M_*(updates.row(d).transpose());
    }

    return true;
  }

protected:

  Eigen::MatrixXd parameters_bias_;          /**< Parameter bias used for computing cost for the test */
  std::vector<double> bias_thresholds_; /**< Threshold to determine whether two trajectories are equal */
  std::vector<double> std_dev_;         /**< Standard deviation used for generating noisy parameters */
  Eigen::MatrixXd smoothing_M_;         /**< Matrix used for smoothing the trajectory */
};

}


#endif /* EXAMPLES_SIMPLE_OPTIMIZATION_TASK_H_ */
