/**
 * @file stomp_example.cpp
 * @brief Demonstrates how to optimize a trajectory using STOMP
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

#include <iostream>
#include <Eigen/Dense>
#include "stomp_core/stomp.h"
#include "simple_optimization_task.h"

using Trajectory = Eigen::MatrixXd;                              /**< Assign Type Trajectory to Eigen::MatrixXd Type */

/**< Declaring optimization variables */
static const std::size_t NUM_DIMENSIONS = 3;                            /**< Number of parameters to optimize */
static const std::size_t NUM_TIMESTEPS = 20;                            /**< Number of timesteps */
static const double DELTA_T = 0.1;                                      /**< Timestep in seconds */
static const std::vector<double> START_POS = {1.4, 1.4, 0.5};           /**< Trajectory starting position */
static const std::vector<double> END_POS = {-1.25, 1.0, -0.26};         /**< Trajectory ending posiiton */
static const std::vector<double> BIAS_THRESHOLD = {0.050,0.050,0.050};  /**< Threshold to determine whether two trajectories are equal */
static const std::vector<double> STD_DEV = {1.0, 1.0, 1.0};             /**< Standard deviation used for generating noisy parameters */

/**
 * @brief Creates a STOMP configuration object with default parameters.
 * @return A STOMP configuration object
 */
stomp_core::StompConfiguration create3DOFConfiguration()
{
  //! [Create Config]
  using namespace stomp_core;

  StompConfiguration c;
  c.num_timesteps = NUM_TIMESTEPS;
  c.num_iterations = 40;
  c.num_dimensions = NUM_DIMENSIONS;
  c.delta_t = DELTA_T;
  c.control_cost_weight = 0.0;
  c.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;
  c.num_iterations_after_valid = 0;
  c.num_rollouts = 20;
  c.max_rollouts = 20;
  //! [Create Config]

  return c;
}

/**
 * @brief Compares whether two trajectories are close to each other within a threshold.
 * @param optimized optimized trajectory
 * @param desired desired trajectory
 * @param thresholds used to determine if two values are equal
 * @return True if the difference between the two is less than the threshold, otherwise false
 */
bool compareDiff(const Trajectory& optimized, const Trajectory& desired,
                 const std::vector<double>& thresholds)
{
  auto num_dimensions = optimized.rows();
  Trajectory diff = Trajectory::Zero(num_dimensions,optimized.cols());
  for(auto d = 0u;d < num_dimensions ; d++)
  {
    diff.row(d) = optimized.row(d)- desired.row(d);
    diff.row(d).cwiseAbs();
    if((diff.row(d).array() > thresholds[d] ).any() )
    {
      return false;
    }
  }

  return true;
}

/**
 * @brief Compute a linear interpolated trajectory given a start and end state
 * @param start start position
 * @param end last position
 * @param num_timesteps number of timesteps
 * @param traj returned linear interpolated trajectory
 */
void interpolate(const std::vector<double>& start, const std::vector<double>& end,
                 std::size_t num_timesteps, Trajectory& traj)
{
  auto dimensions = start.size();
  traj = Eigen::MatrixXd::Zero(dimensions,num_timesteps);
  for(auto d = 0u; d < dimensions; d++)
  {
    double delta = (end[d] - start[d])/(num_timesteps - 1);
    for(auto t = 0u; t < num_timesteps; t++)
    {
      traj(d,t) = start[d] + t*delta;
    }
  }
}

int main(int argc,char** argv)
{
  using namespace stomp_core_examples;
  using namespace stomp_core;

  /**< Creating a Task with a trajectory bias **/
  Trajectory trajectory_bias;
  interpolate(START_POS,END_POS,NUM_TIMESTEPS,trajectory_bias);

  //! [Create Task Object]
  TaskPtr task(new SimpleOptimizationTask(trajectory_bias,BIAS_THRESHOLD,STD_DEV));
  //! [Create Task Object]

  //! [Create STOMP]
  /**< Creating STOMP to find a trajectory close enough to the bias **/
  StompConfiguration config = create3DOFConfiguration();
  Stomp stomp(config,task);
  //! [Create STOMP]

  //! [Solve]
  /**< Optimizing a trajectory close enough to the bias is produced **/
  Trajectory optimized;
  if(stomp.solve(START_POS,END_POS,optimized))
  {
    std::cout<<"STOMP succeeded"<<std::endl;
  }
  else
  {
    std::cout<<"A valid solution was not found"<<std::endl;
    return -1;
  }
  //! [Solve]

  /**< Further verifying the results */
  if(compareDiff(optimized,trajectory_bias,BIAS_THRESHOLD))
  {
    std::cout<<"The solution is within the expected thresholds"<<std::endl;
  }
  else
  {
    std::cout<<"The solution exceeded the required thresholds"<<std::endl;
    return -1;
  }

  return 0;

}




