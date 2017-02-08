/**
 * @file task.h
 * @brief This defines the stomp task
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
#ifndef STOMP_TASK_H_
#define STOMP_TASK_H_

#include <XmlRpcValue.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "stomp_core/utils.h"

namespace stomp_core
{

class Task;
typedef std::shared_ptr<Task> TaskPtr; /**< Defines a boost shared ptr for type Task */

/** @brief Defines the STOMP improvement policy */
class Task
{

public:

    Task(){}

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
    virtual bool generateNoisyParameters(const Eigen::MatrixXd& parameters,
                                         std::size_t start_timestep,
                                         std::size_t num_timesteps,
                                         int iteration_number,
                                         int rollout_number,
                                         Eigen::MatrixXd& parameters_noise,
                                         Eigen::MatrixXd& noise) = 0;

    /**
     * @brief computes the state costs as a function of the noisy parameters for each time step.
     * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param rollout_number    The index of the noisy trajectory whose cost is being evaluated.
     * @param costs vector      A vector containing the state costs per timestep.
     * @param validity          Whether or not the trajectory is valid
     * @return True if cost were properly computed, otherwise false
     */
    virtual bool computeNoisyCosts(const Eigen::MatrixXd& parameters,
                         std::size_t start_timestep,
                         std::size_t num_timesteps,
                         int iteration_number,
                         int rollout_number,
                         Eigen::VectorXd& costs,
                         bool& validity) = 0 ;

    /**
     * @brief computes the state costs as a function of the optimized parameters for each time step.
     * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param costs             A vector containing the state costs per timestep.
     * @param validity          Whether or not the trajectory is valid
     * @return True if cost were properly computed, otherwise false
     */
    virtual bool computeCosts(const Eigen::MatrixXd& parameters,
                         std::size_t start_timestep,
                         std::size_t num_timesteps,
                         int iteration_number,
                         Eigen::VectorXd& costs,
                         bool& validity) = 0 ;

    /**
     * @brief Filters the given noisy parameters which is applied after noisy trajectory generation. It could be used for clipping
     * of joint limits or projecting into the null space of the Jacobian.
     *
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param rollout_number    The rollout index for this noisy parameter set
     * @param parameters        The noisy parameters     *
     * @param filtered          False if no filtering was done
     * @return False if no filtering was done, otherwise true
     */
    virtual bool filterNoisyParameters(std::size_t start_timestep,
                                       std::size_t num_timesteps,
                                       int iteration_number,
                                       int rollout_number,
                                       Eigen::MatrixXd& parameters,
                                       bool& filtered)
    {
      filtered = false;
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
    virtual bool filterParameterUpdates(std::size_t start_timestep,
                                  std::size_t num_timesteps,
                                  int iteration_number,
                                  const Eigen::MatrixXd& parameters,
                                  Eigen::MatrixXd& updates)
    {
      return true;
    }

    /**
     * @brief Called by STOMP at the end of each iteration.
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param cost              The cost value for the current parameters.
     * @param parameters        The value of the parameters at the end of the current iteration [num_dimensions x num_timesteps].
     */
    virtual void postIteration(std::size_t start_timestep,
                                  std::size_t num_timesteps,int iteration_number,double cost,const Eigen::MatrixXd& parameters){}


    /**
     * @brief Called by Stomp at the end of the optimization process
     *
     * @param success           Whether the optimization succeeded
     * @param total_iterations  Number of iterations used
     * @param final_cost        The cost value after optimizing.
     * @param parameters        The parameters generated at the end of the optimization [num_dimensions x num_timesteps]
     */
    virtual void done(bool success,int total_iterations,double final_cost,const Eigen::MatrixXd& parameters){}

};

}
#endif /* TASK_H_ */
