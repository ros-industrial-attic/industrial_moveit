/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

  \file    task.h

  \author  Peter Pastor
  \date    Jun 10, 2010

**********************************************************************/

#ifndef STOMP_TASK_H_
#define STOMP_TASK_H_

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <Eigen/Core>
#include <stomp/covariant_movement_primitive.h>

namespace stomp

{

class Task
{

public:

    Task(){};
    virtual ~Task(){};

    /**
     * Initialize the task for a given number of threads.
     * @param num_threads Number of threads for multi-threading
     * @return
     */
    virtual bool initialize(int num_threads, int num_rollouts) = 0;

    /**
     * Executes the task for the given policy parameters, and returns the costs per timestep
     * Must be thread-safe!
     * @param parameters [num_dimensions] num_parameters - policy parameters to execute
     * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
     * @param weighted_feature_values num_time_steps x num_features matrix of weighted feature values per time step
     * @return
     */
    virtual bool execute(std::vector<Eigen::VectorXd>& parameters,
                         std::vector<Eigen::VectorXd>& projected_parameters,
                         Eigen::VectorXd& costs,
                         Eigen::MatrixXd& weighted_feature_values,
                         const int iteration_number,
                         const int rollout_number,
                         int thread_id,
                         bool compute_gradients,
                         std::vector<Eigen::VectorXd>& gradients,
                         bool& validity) = 0;

    /**
     * Filters the given parameters - for eg, clipping of joint limits
     * Must be thread-safe!
     * @param parameters
     * @return false if no filtering was done
     */
    virtual bool filter(std::vector<Eigen::VectorXd>& parameters, int rollout_id, int thread_id) {return false;};

    /**
     * Get the Policy object of this Task
     * @param policy
     * @return
     */
    virtual bool getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy) = 0;

    /**
     * Sets the Policy object of this Task
     * @param policy
     * @return
     */
    virtual bool setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy) = 0;

    /**
     * Gets the weight of the control cost
     * @return
     */
    virtual double getControlCostWeight() = 0;

    /**
     * Callback executed after each iteration
     */
    virtual void onEveryIteration(){};

};

}
#endif /* TASK_H_ */
