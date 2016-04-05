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
     * Filters the given noisy parameters which is applied after noisy trajectory generation. It could be used for clipping
     * of joint limits or projecting into the null space of the Jacobian.
     *
     * @param parameters
     * @return false if no filtering was done
     */
    virtual bool filterNoisyParameters(Eigen::MatrixXd& parameters,bool& filtered) const
    {
      filtered = false;
      return true;
    };

    /**
     * Filters the given parameters which is applied after the update. It could be used for clipping of joint limits
     * or projecting into the null space of the Jacobian.
     *
     * @param parameters
     * @param filtered false if no filtering was done
     * @return false if there was a failure
     */
    virtual bool filterParameters(Eigen::MatrixXd& parameters,bool& filtered) const
    {
      filtered = false;
      return true;
    };

};

}
#endif /* TASK_H_ */
