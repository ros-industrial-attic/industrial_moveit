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

using Trajectory = Eigen::MatrixXd;

const std::size_t NUM_DIMENSIONS = 3;
const std::size_t NUM_TIMESTEPS = 60;
const double DELTA_T = 0.1;
const std::vector<double> START_POS = {1.4, 1.4, 0.5};
const std::vector<double> END_POS = {-1.25, 1.3, -0.26};
const std::vector<double> BIAS_THRESHOLD = {0.10,0.10,0.10};

using namespace stomp_core;

class DummyTask: public Task
{
public:
  DummyTask(const Trajectory& parameters_bias,
            const std::vector<double>& bias_thresholds):
              parameters_bias_(parameters_bias),
              bias_thresholds_(bias_thresholds)
  {

    // generate smoothing matrix
    int num_timesteps = parameters_bias.cols();
    generateSmoothingMatrix(num_timesteps,1.0,smoothing_M_);

  }

  virtual ~DummyTask(){}

  virtual bool computeCosts(const Trajectory& parameters,
                                        std::size_t start_timestep,
                                        std::size_t num_timesteps,
                                        int iteration_number,
                                        Eigen::VectorXd& costs,
                                        bool& validity) override
  {
    return computeNoisyCosts(parameters,start_timestep,num_timesteps,iteration_number,-1,costs,validity);
  }

  virtual bool computeNoisyCosts(const Trajectory& parameters,
                                        std::size_t start_timestep,
                                        std::size_t num_timesteps,
                                        int iteration_number,
                                        int rollout_number,
                                        Eigen::VectorXd& costs,
                                        bool& validity) override
  {
    costs.setZero(num_timesteps);
    double diff;
    double cost = 0.0d;
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

  virtual bool smoothParameterUpdates(std::size_t start_timestep,
                                      std::size_t num_timesteps,
                                      int iteration_number,
                                      Eigen::MatrixXd& updates) override
  {

    for(auto d = 0u; d < updates.rows(); d++)
    {
      updates.row(d).transpose() = smoothing_M_*(updates.row(d).transpose());
    }

    return true;
  }

protected:

  Trajectory parameters_bias_;
  std::vector<double> bias_thresholds_;
  Eigen::MatrixXd smoothing_M_;
};

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

StompConfiguration create3DOFConfiguration()
{
  StompConfiguration c;
  c.num_timesteps = NUM_TIMESTEPS;
  c.num_iterations = 10;
  c.num_dimensions = NUM_DIMENSIONS;
  c.delta_t = DELTA_T;
  c.control_cost_weight = 0.0;
  c.initialization_method = TrajectoryInitializations::MININUM_CONTROL_COST;
  c.num_iterations_after_valid = 0;
  c.num_rollouts = 20;
  c.max_rollouts = 20;
  c.min_rollouts = 5;

  NoiseGeneration n;
  n.stddev = {0.5, 0.5, 0.5};
  n.decay = {0.9, 0.9, 0.9};
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

  EXPECT_EQ(optimized.rows(),NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(),NUM_TIMESTEPS);
  EXPECT_TRUE(compareDiff(optimized,trajectory_bias,BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

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

  EXPECT_EQ(optimized.rows(),NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(),NUM_TIMESTEPS);
  EXPECT_TRUE(compareDiff(optimized,trajectory_bias,BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout<<line_separator;
  std::cout<<stomp_core::toString(trajectory_bias);
  std::cout<<line_separator;
  std::cout<<toString(optimized)<<"\n";
  std::cout<<"Differences"<<"\n"<<toString(diff)<<line_separator;
}

TEST(Stomp3DOF,solve_stdev_exponential_decay)
{
  Trajectory trajectory_bias;
  int num_timesteps = 40;
  interpolate(START_POS,END_POS,num_timesteps,trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias,BIAS_THRESHOLD));

  StompConfiguration config = create3DOFConfiguration();
  config.initialization_method = TrajectoryInitializations::MININUM_CONTROL_COST;
  config.noise_generation.method = NoiseGeneration::EXPONENTIAL_DECAY;
  config.num_iterations = 20;
  config.num_timesteps = num_timesteps;
  config.delta_t = 0.5;
  Stomp stomp(config,task);

  Trajectory optimized;
  stomp.solve(START_POS,END_POS,optimized);

  EXPECT_EQ(optimized.rows(),NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(),num_timesteps);
  EXPECT_TRUE(compareDiff(optimized,trajectory_bias,BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout<<line_separator;
  std::cout<<stomp_core::toString(trajectory_bias);
  std::cout<<line_separator;
  std::cout<<toString(optimized)<<"\n";
  std::cout<<"Differences"<<"\n"<<toString(diff)<<line_separator;
}

TEST(Stomp3DOF,solve_stdev_constant)
{
  Trajectory trajectory_bias;
  int num_timesteps = 60;
  interpolate(START_POS,END_POS,num_timesteps,trajectory_bias);
  TaskPtr task(new DummyTask(trajectory_bias,BIAS_THRESHOLD));

  StompConfiguration config = create3DOFConfiguration();
  config.initialization_method = TrajectoryInitializations::MININUM_CONTROL_COST;
  config.noise_generation.method = NoiseGeneration::CONSTANT;
  config.num_iterations = 20;
  config.num_timesteps = num_timesteps;
  config.delta_t = 0.1;
  Stomp stomp(config,task);

  Trajectory optimized;
  stomp.solve(START_POS,END_POS,optimized);

  EXPECT_EQ(optimized.rows(),NUM_DIMENSIONS);
  EXPECT_EQ(optimized.cols(),num_timesteps);
  EXPECT_TRUE(compareDiff(optimized,trajectory_bias,BIAS_THRESHOLD));

  // calculate difference
  Trajectory diff;
  diff = trajectory_bias - optimized;

  std::string line_separator = "\n------------------------------------------------------\n";
  std::cout<<line_separator;
  std::cout<<stomp_core::toString(trajectory_bias);
  std::cout<<line_separator;
  std::cout<<toString(optimized)<<"\n";
  std::cout<<"Differences"<<"\n"<<toString(diff)<<line_separator;
}

