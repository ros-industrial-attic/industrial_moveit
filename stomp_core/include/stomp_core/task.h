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

    Task(const std::string& group_name):
      group_name_(group_name)
    {

    };

    virtual ~Task(){};

    /**
     * Initialize the task for a given number of threads.
     * @param structure that contains configuration values
     * @return
     */
    virtual bool initialize(const XmlRpc::XmlRpcValue& params) = 0;


    /**
     * Executes the task for the given policy parameters, and returns the costs per timestep
     * Must be thread-safe!
     * @param parameters [num_dimensions] num_parameters - policy parameters to execute
     * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
     * @param weighted_feature_values num_time_steps x num_features matrix of weighted feature values per time step
     * @return
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

    const std::string& getGroupName() const
    {
      return group_name_;
    }

protected:
    std::string group_name_;

};

}
#endif /* TASK_H_ */
