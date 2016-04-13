/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * task.h
 *
 *  Created on: Mar 7, 2016
 *      Author: Jorge Nicho
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
typedef boost::shared_ptr<Task> TaskPtr;

class Task
{

public:

    Task(){}

    virtual ~Task(){};

    /**
     * @brief computes the state costs as a function of the parameters for each time step.
     * @param parameters [num_dimensions] num_parameters - policy parameters to execute
     * @param costs vector containing the state costs per timestep.
     * @param iteration_number
     * @param rollout_number index of the noisy trajectory whose cost is being evaluated.
     * @param validity whether or not the trajectory is valid
     * @return true if cost were properly computed
     */
    virtual bool computeCosts(const Eigen::MatrixXd& parameters,
                         std::size_t start_timestep,
                         std::size_t num_timesteps,
                         int iteration_number,
                         int rollout_number,
                         Eigen::VectorXd& costs,
                         bool& validity) const = 0 ;

    /**
     * @brief Filters the given noisy parameters which is applied after noisy trajectory generation. It could be used for clipping
     * of joint limits or projecting into the null space of the Jacobian.
     *
     * @param start_timestep    start index into the 'parameters' array, usually 0.
     * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param rollout_number    The rollout index for this noisy parameter set
     * @param parameters        The noisy parameters
     * @return false if no filtering was done
     */
    virtual bool filterNoisyParameters(std::size_t start_timestep,
                                       std::size_t num_timesteps,
                                       int iteration_number,
                                       int rollout_number,
                                       Eigen::MatrixXd& parameters,
                                       bool& filtered) const
    {
      filtered = false;
      return true;
    };

    /**
     * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint limits
     * or projecting into the null space of the Jacobian.
     *
     * @param start_timestep    start index into the 'parameters' array, usually 0.
     * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param parameters        The optimized parameters
     * @param filtered          False if no filtering was done
     * @return                  False if there was a failure
     */
    virtual bool filterParameters(std::size_t start_timestep,
                                  std::size_t num_timesteps,
                                  int iteration_number,
                                  Eigen::MatrixXd& parameters,
                                  bool& filtered) const
    {
      filtered = false;
      return true;
    };


    /**
     * @brief Applies a smoothing scheme to the parameter updates
     *
     * @param start_timestep      start column index in the 'updates' matrix.
     * @param num_timestep        number of column-wise elements to use from the 'updates' matrix.
     * @param iteration_number    the current iteration count.
     * @param updates             the parameter updates.
     * @return                    False if there was a failure, true otherwise.
     */
    virtual bool smoothParameterUpdates(std::size_t start_timestep,
                                        std::size_t num_timesteps,
                                        int iteration_number,
                                        Eigen::MatrixXd& updates) const
    {
      return true;
    }

    /**
     * @brief Called by Stomp at the end of the optimization process
     *
     * @param success           Whether the optimization succeeded
     * @param total_iterations  Number of iterations used
     * @param final_cost        The cost value after optimizing.
     */
    virtual void done(bool success,int total_iterations,double final_cost){}

};

}
#endif /* TASK_H_ */
