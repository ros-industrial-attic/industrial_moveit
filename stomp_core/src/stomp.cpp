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
 * stomp.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Jorge Nicho
 */

#include <ros/console.h>
#include <limits.h>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <math.h>
#include <numeric>
#include "stomp_core/stomp.h"
#include "stomp_core/stomp_core_utils.h"

static const double DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT = 1.0;
static const double EXPONENTIATED_COST_SENSITIVITY = 10;
static const double MIN_COST_DIFFERENCE = 1e-8;
static const double MIN_CONTROL_COST_WEIGHT = 1e-8;

static void computeLinearInterpolation(const std::vector<double>& first,const std::vector<double>& last,
                         int num_timesteps,
                         std::vector<Eigen::VectorXd>& trajectory_joints)
{
  trajectory_joints.resize(first.size(),Eigen::VectorXd::Zero(num_timesteps));
  for(int unsigned i = 0; i < first.size(); i++)
  {
    double dtheta = (last[i] - first[i])/(num_timesteps - 1);
    for(unsigned int j = 0; j < num_timesteps; j++)
    {
      trajectory_joints[i](j) = first[i] + j * dtheta;
    }
  }
}

static void computeCubicInterpolation(const std::vector<double>& first,const std::vector<double>& last,
                         int num_points,double dt,
                         std::vector<Eigen::VectorXd>& trajectory_joints)
{
  std::vector<double> coeffs(4,0);
  double total_time = (num_points - 1) * dt;
  for(int unsigned i = 0; i < first.size(); i++)
  {
    coeffs[0] = first[i];
    coeffs[2] = (3/(pow(total_time,2))) * (last[i] - first[i]);
    coeffs[3] = (-2/(pow(total_time,3))) * (last[i] - first[i]);

    double t;
    for(unsigned j = 0; j < num_points; j++)
    {
      t = j*dt;
      trajectory_joints[i](j) = coeffs[0] + coeffs[2]*pow(t,2) + coeffs[3]*pow(t,3);
    }
  }
}

bool computeMinCostTrajectory(const std::vector<double>& first,
                              const std::vector<double>& last,
                              const Eigen::MatrixXd& smooth_update_matrix,
                              std::vector<Eigen::VectorXd>& trajectory_joints)
{
  using namespace stomp_core;

  if(smooth_update_matrix.rows() != smooth_update_matrix.cols())
  {
    ROS_ERROR("Smoothing Matrix is not square");
    return false;
  }

  int timesteps = smooth_update_matrix.rows();
  Eigen::VectorXd linear_control_cost = Eigen::VectorXd::Zero(timesteps);
  trajectory_joints.resize(first.size(),Eigen::VectorXd::Zero(timesteps));

  for(unsigned int i = 0; i < first.size(); i++)
  {
    linear_control_cost.transpose() = first[i] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        smooth_update_matrix.block(0,0,FINITE_DIFF_RULE_LENGTH - 1,timesteps);

    linear_control_cost.transpose() += last[i] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        smooth_update_matrix.block(timesteps - FINITE_DIFF_RULE_LENGTH + 1,0,FINITE_DIFF_RULE_LENGTH - 1,
                                    timesteps);

    linear_control_cost *=2;

    trajectory_joints[i] = -0.5*smooth_update_matrix*linear_control_cost;

  }

  return true;
}


void computeParametersControlCosts(const std::vector<Eigen::VectorXd>& parameters,
                                          double dt,
                                          double control_cost_weight,
                                          const Eigen::MatrixXd& finite_diff_matrix,
                                          std::vector<Eigen::VectorXd>& control_costs)
{
  Eigen::ArrayXXd Ax;
  for(auto d = 0u; d < parameters.size(); d++)
  {
    Ax = (finite_diff_matrix * (parameters[d])).array();
    control_costs[d] = dt*control_cost_weight*(Ax*Ax).matrix();
  }
}


namespace stomp_core {

Stomp::Stomp(const StompConfiguration& config,TaskPtr task):
    config_(config),
    task_(task),
    proceed_(true),
    parameters_total_cost_(0),
    parameters_valid_(false),
    num_active_rollouts_(0),
    current_iteration_(0)
{

}

Stomp::~Stomp()
{

}

bool Stomp::solve(const std::vector<double>& first,const std::vector<double>& last,
                  std::vector<Eigen::VectorXd>& parameters_optimized)
{
  // initialize trajectory
  if(!computeInitialTrajectory(first,last))
  {
    ROS_ERROR("Unable to generate initial trajectory");
  }

  return solve(parameters_optimized_,parameters_optimized);
}

bool Stomp::solve(const std::vector<Eigen::VectorXd>& initial_parameters,
                  std::vector<Eigen::VectorXd>& parameters_optimized)
{

  // check initial trajectory size
  if(initial_parameters.size() != config_.num_dimensions)
  {
    ROS_ERROR("Initial trajectory dimensions is incorrect");
    return false;
  }
  else
  {
    if(initial_parameters[0].size() != config_.num_timesteps)
    {
      ROS_ERROR("Initial trajectory number of time steps is incorrect");
      return false;
    }
  }

  if(!resetVariables())
  {
    return false;
  }

  setProceed(true);


  current_iteration_ = 0;
  unsigned int valid_iterations = 0;
  double lowest_cost = std::numeric_limits<double>::max();
  while(current_iteration_ < config_.num_iterations && runSingleIteration())
  {

    if(parameters_valid_)
    {
      valid_iterations++;
      ROS_DEBUG("Found valid solution, will iterate %i more times ",
               config_.num_iterations_after_valid - valid_iterations);
    }
    else
    {
      valid_iterations = 0;
    }

    if(valid_iterations>= config_.num_iterations_after_valid)
    {
      break;
    }

    if(parameters_total_cost_ < lowest_cost)
    {
      lowest_cost = parameters_total_cost_;
    }

    ROS_DEBUG("STOMP completed iteration %i with cost %f",current_iteration_,lowest_cost);

    current_iteration_++;
  }

  if(parameters_valid_)
  {
    ROS_INFO("STOMP found a valid solution with cost %f after %i iterations",
             lowest_cost,current_iteration_);
  }
  else
  {
    ROS_ERROR("STOMP failed to find a valid solution after %i iterations",current_iteration_);
  }

  parameters_optimized = parameters_optimized_;

  return parameters_valid_;
}

bool Stomp::resetVariables()
{
  // generate finite difference matrix
  generateFiniteDifferenceMatrix(config_.num_timesteps,DerivativeOrders::STOMP_ACCELERATION,
                                 config_.delta_t,finite_diff_matrix_A_);

  // control cost matrix
  control_cost_matrix_R_ = finite_diff_matrix_A_.transpose() * finite_diff_matrix_A_;
  inv_control_cost_matrix_R_ = control_cost_matrix_R_.fullPivLu().inverse();

  // projection matrix
  projection_matrix_M_ = inv_control_cost_matrix_R_;
  double max = 0;
  for(auto t = 0u; t < config_.num_timesteps; t++)
  {
    max = projection_matrix_M_(t,t);
    projection_matrix_M_.col(t)*= (1.0/(config_.num_timesteps*max)); // scaling such that the maximum value is 1/num_timesteps
  }
  inv_projection_matrix_M_ = projection_matrix_M_.fullPivLu().inverse();

  // noise generation
  mv_gaussian_.reset(new MultivariateGaussian(Eigen::VectorXd::Zero(config_.num_timesteps),inv_control_cost_matrix_R_));
  noise_stddevs_ = config_.noise_generation.stddev;
  temp_noise_array_ =  Eigen::VectorXd::Zero(config_.num_timesteps);

  // noisy rollouts allocation
  int d = config_.num_dimensions;
  num_active_rollouts_ = 0;
  noisy_rollouts_.resize(config_.max_rollouts);
  reused_rollouts_.resize(config_.max_rollouts);

  // initializing rollout
  Rollout rollout;
  rollout.noise.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.parameters_noise.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.parameters_noise_projected.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));

  rollout.probabilities.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.full_probabilities.resize(d);

  rollout.full_costs.resize(d);
  rollout.cumulative_costs.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.control_costs.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.total_costs.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.state_costs = Eigen::VectorXd::Zero(config_.num_timesteps);
  rollout.importance_weight = DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT;

  for(unsigned int r = 0; r < config_.max_rollouts ; r++)
  {
    noisy_rollouts_[r] = rollout;
    reused_rollouts_[r] = rollout;
  }

  // parameter updates
  parameters_control_costs_.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  parameters_state_costs_ = Eigen::VectorXd::Zero(config_.num_timesteps);
  temp_parameter_updates_ = Eigen::VectorXd::Zero(config_.num_timesteps);

  return true;
}

void Stomp::updateNoiseStddev()
{
  std::vector<double>& stddev = config_.noise_generation.stddev;
  std::vector<double>& decay = config_.noise_generation.decay;
  switch(config_.noise_generation.method)
  {
    case NoiseGeneration::ADAPTIVE:
    {
      double denom;
      double numer;
      double frob_stddev;
      double update_rate = config_.noise_generation.update_rate;
      const std::vector<double>& min_stddev = config_.noise_generation.min_stddev;
      for(unsigned int d = 0; d < config_.num_dimensions; d++)
      {
        denom = 0;
        numer = 0;
        for (auto r = 0u; r<num_active_rollouts_; ++r)
        {
            denom += noisy_rollouts_[r].full_probabilities[d];
            numer += noisy_rollouts_[r].full_probabilities[d] *
                    double(noisy_rollouts_[r].noise[d].transpose() * control_cost_matrix_R_ * noisy_rollouts_[r].noise[d]);
        }
        frob_stddev =  sqrt(numer/(denom*config_.num_timesteps));
        frob_stddev = std::isnan(frob_stddev) ? 0 : frob_stddev;

        if(!std::isnan(frob_stddev))
        {
          noise_stddevs_[d] = (1.0 - update_rate) * noise_stddevs_[d] + update_rate * frob_stddev;
        }

        if (noise_stddevs_[d] < min_stddev[d])
          noise_stddevs_[d] = min_stddev[d];

      }
    }

      break;
    case NoiseGeneration::CONSTANT:

      noise_stddevs_ = stddev;
      break;
    case NoiseGeneration::EXPONENTIAL_DECAY:

      noise_stddevs_.resize(stddev.size());
      for(unsigned int i = 0; i < stddev.size() ; i++)
      {
        noise_stddevs_[i] = stddev[i]*pow(decay[i],current_iteration_);
      }
      break;
  }
}

bool Stomp::computeInitialTrajectory(const std::vector<double>& first,const std::vector<double>& last)
{
  // allocating
  parameters_optimized_.resize(config_.num_dimensions,Eigen::VectorXd::Zero(config_.num_timesteps));
  bool valid = true;

  switch(config_.initialization_method)
  {
    case TrajectoryInitializations::CUBIC_POLYNOMIAL_INTERPOLATION:

      computeCubicInterpolation(first,last,config_.num_timesteps,config_.delta_t,parameters_optimized_);
      break;
    case TrajectoryInitializations::LINEAR_INTERPOLATION:

      computeLinearInterpolation(first,last,config_.num_timesteps,parameters_optimized_);
      break;
    case TrajectoryInitializations::MININUM_CONTROL_COST:

      valid = computeMinCostTrajectory(first,last,inv_control_cost_matrix_R_,parameters_optimized_);
      break;
  }

  return valid;
}

bool Stomp::cancel()
{
  ROS_WARN("Interrupting STOMP");
  setProceed(false);
}

bool Stomp::runSingleIteration()
{
  if(!getProceed())
  {
    return false;
  }

  // updates using previous iteration results
  updateNoiseStddev();

  bool proceed = generateNoisyRollouts() &&
      computeNoisyRolloutsCosts() &&
      filterNoisyRollouts() &&
      computeProbabilities() &&
      updateParameters() &&
      filterUpdatedParameters() &&
      computeOptimizedCost();

  return proceed;
}

bool Stomp::generateNoisyRollouts()
{
  // calculating number of rollouts to reuse from previous iteration
  std::vector< std::pair<double,int> > rollout_cost_sorter; // Used to sort noisy trajectories in ascending order wrt their total cost
  double h = EXPONENTIATED_COST_SENSITIVITY;
  int rollouts_stored = num_active_rollouts_;
  int rollouts_generate = config_.num_rollouts_per_iteration;
  int rollouts_total = rollouts_generate + rollouts_stored;
  int rollouts_reuse =  rollouts_total < config_.max_rollouts  ? rollouts_stored:  config_.max_rollouts - rollouts_generate ;

  // selecting least costly rollouts from previous iteration
  if(rollouts_reuse > 0)
  {
    // find min and max cost for exponential cost scaling
    double min_cost = std::numeric_limits<double>::max();
    double max_cost = std::numeric_limits<double>::min();
    for (int r=1; r<rollouts_stored; ++r)
    {
      double c = noisy_rollouts_[r].total_cost;
      if (c < min_cost)
        min_cost = c;
      if (c > max_cost)
        max_cost = c;
    }

    double cost_denom = max_cost - min_cost;
    if (cost_denom < 1e-8)
      cost_denom = 1e-8;

    // compute weighted cost on all rollouts
    for (auto r = 0u; r<rollouts_stored; ++r)
    {

      // Apply noise generated on the previous iteration onto the current trajectory
      for (auto d = 0u; d<config_.num_dimensions; ++d)
      {
        noisy_rollouts_[r].noise[d] = inv_projection_matrix_M_ * (
            noisy_rollouts_[r].parameters_noise_projected[d] - parameters_optimized_[d]);

        noisy_rollouts_[r].parameters_noise[d] =  parameters_optimized_[d] + noisy_rollouts_[r].noise[d];
      }

      double cost_prob = exp(-h*(noisy_rollouts_[r].total_cost - min_cost)/cost_denom);
      double weighted_cost = cost_prob * noisy_rollouts_[r].importance_weight;
      rollout_cost_sorter.push_back(std::make_pair(-weighted_cost,r));
    }

    std::sort(rollout_cost_sorter.begin(), rollout_cost_sorter.end());

    // use the best ones: (copy them into reused_rollouts)
    for (auto r = 0u; r<rollouts_stored; ++r)
    {
      int reuse_index = rollout_cost_sorter[r].second;
      reused_rollouts_[r] = noisy_rollouts_[reuse_index];
    }

    // copy them back from reused_rollouts_ into rollouts_
    for (auto r = 0u; r<rollouts_reuse; ++r)
    {
      noisy_rollouts_[rollouts_generate+r] = reused_rollouts_[r];
    }
  }

  // generate new noisy rollouts
  for (auto d = 0u; d<config_.num_dimensions; ++d)
  {
    for(auto r = 0u; r < rollouts_generate; r++)
    {
      mv_gaussian_->sample(temp_noise_array_);
      noisy_rollouts_[r].noise[d]  = noise_stddevs_[d] * temp_noise_array_;
      noisy_rollouts_[r].parameters_noise[d] = parameters_optimized_[d] + noisy_rollouts_[r].noise[d];
    }

  }

  // update total active rollouts
  num_active_rollouts_ = rollouts_reuse + rollouts_generate;

  return true;
}

bool Stomp::filterNoisyRollouts()
{
  // apply post noise generation filters
  bool proceed = true;
  for(auto r = 0u ; r < num_active_rollouts_; r++)
  {
    if(task_->filterNoisyParameters(noisy_rollouts_[r].parameters_noise))
    {
      // compute projected noisy trajectories
      for(auto d  = 0u; d < config_.num_dimensions; d++)
      {
        noisy_rollouts_[r].parameters_noise_projected[d] = projection_matrix_M_*(
            noisy_rollouts_[r].parameters_noise[d] - parameters_optimized_[d]);
      }
    }
    else
    {
      proceed = false;
      break;
    }
  }

  return proceed;
}

bool Stomp::computeNoisyRolloutsCosts()
{
  // convenience anonymous function for adding costs arrays
  auto sum_array_func = [](const double& val,const Eigen::VectorXd& costs)
      {
        return costs.sum();
      };

  // computing state and control costs
  bool valid = computeRolloutsStateCosts() && computeRolloutsControlCosts();

  if(valid)
  {
    // compute total costs
    double total_state_cost ;
    double total_control_cost;

    for(auto r = 0u ; r < num_active_rollouts_;r++)
    {
      Rollout& rollout = noisy_rollouts_[r];
      total_state_cost = rollout.state_costs.sum();

      // Compute control + state cost for each joint
      for(auto d = 0u; d < config_.num_dimensions; d++)
      {
         rollout.full_costs[d] = rollout.control_costs[d].sum() + total_state_cost;
      }

      // Compute total control cost for the entire trajectory
      rollout.total_cost = rollout.state_costs.sum() +
          std::accumulate(rollout.control_costs.begin(),rollout.control_costs.end(),0.0d,sum_array_func);

      // Compute total cost for each time step
      for(auto d = 0u; d < config_.num_dimensions; d++)
      {
        rollout.total_costs[d] = rollout.state_costs + rollout.control_costs[d];
        rollout.cumulative_costs[d] = Eigen::VectorXd::Ones(config_.num_timesteps)*rollout.total_costs[d];
      }
    }
  }

  return valid;
}

bool Stomp::computeRolloutsStateCosts()
{

  bool all_valid = true;
  bool proceed = true;
  for(auto r = 0u ; r < config_.num_rollouts_per_iteration && getProceed(); r++)
  {
    if(!getProceed())
    {
      proceed = false;
      break;
    }

    Rollout& rollout = noisy_rollouts_[r];
    if(!task_->computeCosts(rollout.parameters_noise,rollout.state_costs,current_iteration_,r,all_valid))
    {
      ROS_ERROR("Trajectory cost computation failed for rollout %i.",r);
      proceed = false;
      break;
    }
  }

  return proceed;
}
bool Stomp::computeRolloutsControlCosts()
{
  Eigen::ArrayXXd Ax; // accelerations
  for(auto r = 0u ; r < num_active_rollouts_; r++)
  {
    Rollout& rollout = noisy_rollouts_[r];

    if(config_.control_cost_weight < MIN_CONTROL_COST_WEIGHT)
    {
      for(auto d = 0u; d < config_.num_dimensions; d++)
      {
        rollout.control_costs[d].setZero();
      }
    }
    else
    {
      computeParametersControlCosts(rollout.parameters_noise_projected,
                                    config_.delta_t,
                                    config_.control_cost_weight,
                                    finite_diff_matrix_A_,rollout.control_costs);
    }
  }
  return true;
}
/*
void Stomp::computeParametersControlCosts(const std::vector<Eigen::VectorXd>& parameters,
                                          double dt,
                                          double control_cost_weight,
                                          const Eigen::MatrixXd& finite_diff_matrix,
                                          std::vector<Eigen::VectorXd>& control_costs)
{
  for(auto d = 0u; d < parameters.size(); d++)
  {
    Ax = (finite_diff_matrix * (parameters[d])).array();
    control_costs[d] = dt*control_cost_weight*(Ax*Ax).matrix();
  }
}*/

bool Stomp::computeProbabilities()
{

  double min_cost;
  double max_cost;

  double denom;
  double numerator;
  double probl_sum = 0.0; // total probability sum of all rollouts for each joint
  const double h = EXPONENTIATED_COST_SENSITIVITY;

  for (auto d = 0u; d<config_.num_dimensions; ++d)
  {
    // find min and max cost over all rollouts for joint d:
    min_cost = noisy_rollouts_[0].cumulative_costs[d].minCoeff();
    max_cost = noisy_rollouts_[0].cumulative_costs[d].maxCoeff();
    for (auto r = 0u; r<num_active_rollouts_; ++r)
    {
      double min_r = noisy_rollouts_[r].cumulative_costs[d].minCoeff();
      double max_r = noisy_rollouts_[r].cumulative_costs[d].maxCoeff();
      if (min_cost > min_r)
        min_cost = min_r;
      if (max_cost < max_r)
        max_cost = max_r;
    }

    denom = max_cost - min_cost; //

    for (auto t = 0u; t<config_.num_timesteps; t++)
    {

      // prevent division by zero:
      if (denom < MIN_COST_DIFFERENCE)
      {
        denom = MIN_COST_DIFFERENCE;
      }

      probl_sum = 0.0;
      for (auto r = 0u; r<num_active_rollouts_; ++r)
      {
          // this is the exponential term in the probability calculation described in the literature
          noisy_rollouts_[r].probabilities[d](t) = noisy_rollouts_[r].importance_weight *
              exp(-h*(noisy_rollouts_[r].cumulative_costs[d](t) - min_cost)/denom);

          probl_sum += noisy_rollouts_[r].probabilities[d](t);
      }

      // scaling each probability value by the sum of all probabilities corresponding to all rollouts at time "t"
      for (auto r = 0u; r<num_active_rollouts_; ++r)
      {
        noisy_rollouts_[r].probabilities[d](t) /= probl_sum;
      }
    }
  }

  return true;
}

bool Stomp::updateParameters()
{

  for(auto d = 0u; d < config_.num_dimensions ; d++)
  {

    temp_parameter_updates_.setZero();
    for(auto& rollout: noisy_rollouts_)
    {
      temp_parameter_updates_ += (rollout.noise[d].array() * rollout.probabilities[d].array()).matrix();
    }

    parameters_optimized_[d] = projection_matrix_M_ * temp_parameter_updates_;
  }

  return true;

}

bool Stomp::filterUpdatedParameters()
{
  return task_->filterParameters(parameters_optimized_);
}

bool Stomp::computeOptimizedCost()
{
  bool proceed = true;

  // control costs
  parameters_total_cost_ = 0;
  if(config_.control_cost_weight > MIN_CONTROL_COST_WEIGHT)
  {
    computeParametersControlCosts(parameters_optimized_,
                                  config_.delta_t,
                                  config_.control_cost_weight,
                                  finite_diff_matrix_A_,
                                  parameters_control_costs_);

    // adding all costs
    for(auto& costs: parameters_control_costs_)
    {
      parameters_total_cost_ += costs.sum();
    }
  }

  // state costs
  if(task_->computeCosts(parameters_optimized_,
                         parameters_state_costs_,current_iteration_,0,parameters_valid_))
  {


    parameters_total_cost_ += parameters_state_costs_.sum();
  }
  else
  {
    proceed = false;
  }

  return proceed;
}

void Stomp::setProceed(bool proceed)
{
  proceed_mutex_.lock();
  proceed_ = proceed;
  proceed_mutex_.unlock();
}

bool Stomp::getProceed()
{
  bool proceed;
  proceed_mutex_.lock();
  proceed = proceed_;
  proceed_mutex_.unlock();
  return proceed;
}


} /* namespace stomp */
