/*
 * stomp_3dof.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: Jorge Nicho
 */

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

#include <iostream>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "stomp_core/stomp.h"
#include "stomp_core/task.h"

using Trajectory = std::vector<Eigen::VectorXd>;

const std::size_t NUM_DIMENSIONS = 3;
const std::size_t NUM_TIMESTEPS = 15;
const double DELTA_T = 0.2;
const std::vector<double> START_POS = {1.4, 1.4, 0.5};
const std::vector<double> END_POS = {-1.25, 1.3, -0.26};
const std::vector<double> BIAS_THRESHOLD = {0.1,0.10,0.10};

using namespace stomp_core;

class DummyTask: public Task
{
public:
  DummyTask(const std::vector<Eigen::VectorXd>& parameters_bias,
            const std::vector<double>& bias_thresholds):
              parameters_bias_(parameters_bias),
              bias_thresholds_(bias_thresholds)
  {

  }

  virtual ~DummyTask(){}

  virtual bool computeCosts(const std::vector<Eigen::VectorXd>& parameters,
                                        std::size_t start_timestep,
                                        std::size_t num_timesteps,
                                        int iteration_number,
                                        int rollout_number,
                                        Eigen::VectorXd& costs,
                                        bool& validity) const
  {
    costs.setZero(num_timesteps);
    double diff;
    double cost = 0.0d;
    validity = true;

    for(std::size_t t = 0u; t < num_timesteps; t++)
    {
      cost = 0;
      for(std::size_t d = 0u; d < parameters.size() ; d++)
      {

        diff = std::abs(parameters[d](t) - parameters_bias_[d](t));
        if( diff > std::abs(bias_thresholds_[d]))
        {
          cost += diff;
          validity = false;
        }
      }

      costs(t) = cost;
    }


    // scaling costs to the maximum
    double max = costs.maxCoeff();
    costs /= (max < 1e-8 ? 1:max);
    return true;
  }

protected:

  std::vector<Eigen::VectorXd> parameters_bias_;
  std::vector<double> bias_thresholds_;
};

StompConfiguration create3DOFConfiguration()
{
  StompConfiguration c;
  c.num_timesteps = NUM_TIMESTEPS;
  c.num_iterations = 10;
  c.num_dimensions = NUM_DIMENSIONS;
  c.delta_t = DELTA_T;
  c.control_cost_weight = 0;
  c.initialization_method = TrajectoryInitializations::MININUM_CONTROL_COST;
  c.num_iterations_after_valid = 1;
  c.num_rollouts_per_iteration = 10;
  c.max_rollouts = 10;
  c.min_rollouts = 10;

  NoiseGeneration n;
  n.stddev = {0.5, 0.5, 0.5};
  n.decay = {1.0, 1.0, 1.0};
  n.min_stddev = {0.05, 0.05, 0.05};
  n.method = NoiseGeneration::ADAPTIVE;
  n.update_rate = 0.2;
  c.noise_generation = n;

  return c;
}

void interpolate(const std::vector<double>& start, const std::vector<double>& end,
                 std::size_t num_timesteps, Trajectory& traj)
{
  auto dimensions = start.size();
  traj.resize(dimensions,Eigen::VectorXd::Zero(num_timesteps));
  for(auto d = 0u; d < dimensions; d++)
  {
    double delta = (end[d] - start[d])/(num_timesteps - 1);
    for(auto t = 0u; t < num_timesteps; t++)
    {
      traj[d](t) = start[d] + t*delta;
    }
  }
}


TEST(Stomp3DOF,construction)
{
  Trajectory trajectory_bias;
  interpolate(START_POS,END_POS,NUM_TIMESTEPS,trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias,BIAS_THRESHOLD));

  Stomp stomp(create3DOFConfiguration(),task);
}

TEST(Stomp3DOF,solve_default)
{
  Trajectory trajectory_bias;
  interpolate(START_POS,END_POS,NUM_TIMESTEPS,trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias,BIAS_THRESHOLD));

  StompConfiguration config = create3DOFConfiguration();
  Stomp stomp(config,task);

  Trajectory optimized;
  stomp.solve(START_POS,END_POS,optimized);

  EXPECT_EQ(optimized.size(),NUM_DIMENSIONS);
  for(auto d = 0u; d < NUM_DIMENSIONS;d++)
  {
    EXPECT_EQ(optimized[d].size(),NUM_TIMESTEPS);
  }

  // calculate difference
  Trajectory diff(config.num_dimensions,Eigen::VectorXd::Zero(config.num_timesteps));
  for(auto d = 0u; d < config.num_dimensions ; d++)
  {
    diff[d] = trajectory_bias[d] - optimized[d];
  }

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout<<line_separator;
  std::cout<<stomp_core::toString(trajectory_bias);
  std::cout<<line_separator;
  std::cout<<toString(optimized)<<"\n";
  std::cout<<"Differences"<<"\n"<<toString(diff)<<line_separator;
}

TEST(Stomp3DOF,solve_interpolated_initial)
{
  Trajectory trajectory_bias;
  interpolate(START_POS,END_POS,NUM_TIMESTEPS,trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias,BIAS_THRESHOLD));

  StompConfiguration config = create3DOFConfiguration();
  config.initialization_method = TrajectoryInitializations::LINEAR_INTERPOLATION;
  Stomp stomp(config,task);

  Trajectory optimized;
  stomp.solve(trajectory_bias,optimized);

  EXPECT_EQ(optimized.size(),NUM_DIMENSIONS);
  for(auto d = 0u; d < NUM_DIMENSIONS;d++)
  {
    EXPECT_EQ(optimized[d].size(),NUM_TIMESTEPS);
  }

  // calculate difference
  Trajectory diff(config.num_dimensions,Eigen::VectorXd::Zero(config.num_timesteps));
  for(auto d = 0u; d < config.num_dimensions ; d++)
  {
    diff[d] = trajectory_bias[d] - optimized[d];
  }

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout<<line_separator;
  std::cout<<stomp_core::toString(trajectory_bias);
  std::cout<<line_separator;
  std::cout<<toString(optimized)<<"\n";
  std::cout<<"Differences"<<"\n"<<toString(diff)<<line_separator;
}

