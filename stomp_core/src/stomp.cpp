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
#include <stomp_core/utils.h>
#include <numeric>
#include "stomp_core/stomp.h"

static const double DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT = 1.0;
static const double EXPONENTIATED_COST_SENSITIVITY = 10;
static const double MIN_COST_DIFFERENCE = 1e-8;
static const double MIN_CONTROL_COST_WEIGHT = 1e-8;
static const double OPTIMIZATION_TIMESTEP = 1;

static void computeLinearInterpolation(const std::vector<double>& first,const std::vector<double>& last,
                         int num_timesteps,
                         Eigen::MatrixXd& trajectory_joints)
{
  trajectory_joints.setZero(first.size(),num_timesteps);
  for(int unsigned i = 0; i < first.size(); i++)
  {
    double dtheta = (last[i] - first[i])/(num_timesteps - 1);
    for(unsigned int j = 0; j < num_timesteps; j++)
    {
      trajectory_joints(i,j) = first[i] + j * dtheta;
    }
  }
}

static void computeCubicInterpolation(const std::vector<double>& first,const std::vector<double>& last,
                         int num_points,double dt,
                         Eigen::MatrixXd& trajectory_joints)
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
      trajectory_joints(i,j) = coeffs[0] + coeffs[2]*pow(t,2) + coeffs[3]*pow(t,3);
    }
  }
}

bool computeMinCostTrajectory(const std::vector<double>& first,
                              const std::vector<double>& last,
                              const Eigen::MatrixXd& control_cost_matrix_R_padded,
                              const Eigen::MatrixXd& inv_control_cost_matrix_R,
                              Eigen::MatrixXd& trajectory_joints)
{
  using namespace stomp_core;

  if(control_cost_matrix_R_padded.rows() != control_cost_matrix_R_padded.cols())
  {
    ROS_ERROR("Control Cost Matrix is not square");
    return false;
  }


  int timesteps = control_cost_matrix_R_padded.rows() - 2*(FINITE_DIFF_RULE_LENGTH - 1);
  int start_index_padded = FINITE_DIFF_RULE_LENGTH - 1;
  int end_index_padded = start_index_padded + timesteps-1;
  std::vector<Eigen::VectorXd> linear_control_cost(first.size(),Eigen::VectorXd::Zero(timesteps));
  trajectory_joints.setZero(first.size(),timesteps);


  for(unsigned int d = 0; d < first.size(); d++)
  {
    linear_control_cost[d].transpose() = first[d] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        control_cost_matrix_R_padded.block(0,start_index_padded,FINITE_DIFF_RULE_LENGTH - 1,timesteps);

    linear_control_cost[d].transpose() += last[d] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        control_cost_matrix_R_padded.block(end_index_padded + 1,start_index_padded,FINITE_DIFF_RULE_LENGTH - 1,
                                    timesteps);

    linear_control_cost[d] *=2;

    trajectory_joints.row(d) = -0.5*inv_control_cost_matrix_R*linear_control_cost[d];

  }

  return true;
}


void computeParametersControlCosts(const Eigen::MatrixXd& parameters,
                                          double dt,
                                          double control_cost_weight,
                                          const Eigen::MatrixXd& control_cost_matrix_R,
                                          Eigen::MatrixXd& control_costs)
{
  std::size_t num_timesteps = parameters.cols();
  double cost = 0;
  for(auto d = 0u; d < parameters.rows(); d++)
  {
    cost = double(parameters.row(d)*(control_cost_matrix_R*parameters.row(d).transpose()));
    control_costs.row(d).setConstant( 0.5*(1/dt)*cost );
  }

  double max_coeff = control_costs.maxCoeff();
  control_costs /= (max_coeff > 1e-8) ? max_coeff : 1;
  control_costs *= control_cost_weight;
}


namespace stomp_core {

bool Stomp::parseConfig(XmlRpc::XmlRpcValue config,StompConfiguration& stomp_config)
{
  using namespace XmlRpc;

  try
  {

    stomp_config.control_cost_weight = static_cast<double>(config["control_cost_weight"]);
    stomp_config.delta_t = static_cast<double>(config["delta_t"]);
    stomp_config.initialization_method = static_cast<int>(config["initialization_method"]);
    stomp_config.max_rollouts = static_cast<int>(config["max_rollouts"]);
    stomp_config.min_rollouts = static_cast<int>(config["min_rollouts"]);
    stomp_config.num_dimensions = static_cast<int>(config["num_dimensions"]);
    stomp_config.num_iterations = static_cast<int>(config["num_iterations"]);
    stomp_config.num_iterations_after_valid = static_cast<int>(config["num_iterations_after_valid"]);
    stomp_config.num_rollouts = static_cast<int>(config["num_rollouts"]);
    stomp_config.num_timesteps = static_cast<int>(config["num_timesteps"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("Failed to parse Stomp configuration ");
    return false;
  }

  try
  {

    // noise parameters
    XmlRpc::XmlRpcValue noisegen, array;
    noisegen = config["noise_generation"];

    // stddev
    std::vector<double> vals;
    std::map<std::string,std::vector<double> > array_entries = {{"stddev", vals},{"decay", vals},{"min_stddev",vals}};
    for(auto& entry: array_entries)
    {
      array = noisegen[entry.first];
      vals.clear();
      for(auto i = 0u; i < array.size(); i++)
      {
        vals.push_back(static_cast<double>(array[i]));
      }
      array_entries[entry.first] = vals;
    }

    stomp_config.noise_generation.stddev = array_entries["stddev"];
    stomp_config.noise_generation.decay = array_entries["decay"];
    stomp_config.noise_generation.min_stddev = array_entries["min_stddev"];

    stomp_config.noise_generation.method = static_cast<int>(noisegen["method"]);
    stomp_config.noise_generation.update_rate = static_cast<double>(noisegen["update_rate"]);

  }
  catch(XmlRpc::XmlRpcException& e)
  {
    std::stringstream ss;
    config.write(ss);
    ROS_ERROR("Failed to parse Stomp 'noise_generation' entry from parameter %s",ss.str().c_str());
    return false;
  }

  return true;
}


Stomp::Stomp(const StompConfiguration& config,TaskPtr task):
    config_(config),
    task_(task),
    proceed_(true),
    parameters_total_cost_(0),
    parameters_valid_(false),
    num_active_rollouts_(0),
    current_iteration_(0)
{

  resetVariables();

}

Stomp::~Stomp()
{

}

bool Stomp::solve(const std::vector<double>& first,const std::vector<double>& last,
                  Eigen::MatrixXd& parameters_optimized)
{
  // initialize trajectory
  if(!computeInitialTrajectory(first,last))
  {
    ROS_ERROR("Unable to generate initial trajectory");
  }

  return solve(parameters_optimized_,parameters_optimized);
}

bool Stomp::solve(const Eigen::MatrixXd& initial_parameters,
                  Eigen::MatrixXd& parameters_optimized)
{
  if(parameters_optimized_.isZero())
  {
    parameters_optimized_ = initial_parameters;
  }

  // check initial trajectory size
  if(initial_parameters.rows() != config_.num_dimensions || initial_parameters.cols() != config_.num_timesteps)
  {
    ROS_ERROR("Initial trajectory dimensions is incorrect");
    return false;
  }
  else
  {
    if(initial_parameters.cols() != config_.num_timesteps)
    {
      ROS_ERROR("Initial trajectory number of time steps is incorrect");
      return false;
    }
  }

  setProceed(true);

  current_iteration_ = 1;
  unsigned int valid_iterations = 0;
  double lowest_cost = std::numeric_limits<double>::max();
  while(current_iteration_ <= config_.num_iterations && runSingleIteration())
  {

    lowest_cost = parameters_total_cost_ < lowest_cost ? parameters_total_cost_ : lowest_cost;

    ROS_DEBUG("STOMP completed iteration %i with cost %f",current_iteration_,lowest_cost);


    if(parameters_valid_)
    {
      ROS_DEBUG("Found valid solution, will iterate %i more time(s) ",
               config_.num_iterations_after_valid - valid_iterations);

      valid_iterations++;
    }
    else
    {
      valid_iterations = 0;
    }

    if(valid_iterations > config_.num_iterations_after_valid)
    {
      break;
    }

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

  // notifying task
  task_->done(parameters_valid_,current_iteration_,lowest_cost);

  return parameters_valid_;
}

bool Stomp::resetVariables()
{
  // verifying configuration
  if(config_.max_rollouts < config_.num_rollouts)
  {
    ROS_WARN_STREAM("'max_rollouts' must be greater than 'num_rollouts_per_iteration'.");
    config_.max_rollouts = config_.num_rollouts;
  }

  if(config_.min_rollouts > config_.num_rollouts)
  {
    ROS_WARN_STREAM("'min_rollouts' must be less than 'num_rollouts_per_iteration'");
    config_.min_rollouts = config_.num_rollouts;
  }

  // generate finite difference matrix
  start_index_padded_ = FINITE_DIFF_RULE_LENGTH-1;
  num_timesteps_padded_ = config_.num_timesteps + 2*(FINITE_DIFF_RULE_LENGTH-1);
  generateFiniteDifferenceMatrix(num_timesteps_padded_,DerivativeOrders::STOMP_ACCELERATION,
                                 OPTIMIZATION_TIMESTEP,finite_diff_matrix_A_padded_);

  /* control cost matrix (R = A_transpose * A):
   * Note: Original code multiplies the A product by the time interval.  However this is not
   * what was described in the literature
   */
  control_cost_matrix_R_padded_ = OPTIMIZATION_TIMESTEP*finite_diff_matrix_A_padded_.transpose() * finite_diff_matrix_A_padded_;
  control_cost_matrix_R_ = control_cost_matrix_R_padded_.block(
      start_index_padded_,start_index_padded_,config_.num_timesteps,config_.num_timesteps);
  inv_control_cost_matrix_R_ = control_cost_matrix_R_.fullPivLu().inverse();

  /*
   * Applying scale factor to ensure that max(R^-1)==1
   */
  double maxVal = std::abs(inv_control_cost_matrix_R_.maxCoeff());
  control_cost_matrix_R_padded_ *= maxVal;
  control_cost_matrix_R_ *= maxVal;
  inv_control_cost_matrix_R_ /= maxVal;

  /* Noise Generation*/
  random_dist_generators_.clear();
  for(auto d = 0; d < config_.num_dimensions ; d++)
  {
    random_dist_generators_.push_back(
        MultivariateGaussianPtr(new MultivariateGaussian(Eigen::VectorXd::Zero(config_.num_timesteps),
                                              inv_control_cost_matrix_R_)));
  }
  noise_stddevs_ = config_.noise_generation.stddev;
  temp_noise_array_ =  Eigen::VectorXd::Zero(config_.num_timesteps);

  // noisy rollouts allocation
  int d = config_.num_dimensions;
  num_active_rollouts_ = 0;
  noisy_rollouts_.resize(config_.max_rollouts);
  reused_rollouts_.resize(config_.max_rollouts);

  // initializing rollout
  Rollout rollout;
  rollout.noise = Eigen::MatrixXd::Zero(d, config_.num_timesteps);
  rollout.parameters_noise = Eigen::MatrixXd::Zero(d, config_.num_timesteps);

  rollout.probabilities= Eigen::MatrixXd::Zero(d, config_.num_timesteps);
  rollout.full_probabilities.resize(d);

  rollout.full_costs.resize(d);
  rollout.control_costs = Eigen::MatrixXd::Zero(d, config_.num_timesteps);
  rollout.total_costs= Eigen::MatrixXd::Zero(d, config_.num_timesteps);
  rollout.state_costs = Eigen::VectorXd::Zero(config_.num_timesteps);
  rollout.importance_weight = DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT;

  for(unsigned int r = 0; r < config_.max_rollouts ; r++)
  {
    noisy_rollouts_[r] = rollout;
    reused_rollouts_[r] = rollout;
  }

  // parameter updates
  parameters_updates_ = Eigen::MatrixXd::Zero(d, config_.num_timesteps);
  parameters_control_costs_= Eigen::MatrixXd::Zero(d, config_.num_timesteps);
  parameters_state_costs_ = Eigen::VectorXd::Zero(config_.num_timesteps);

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
                    double( noisy_rollouts_[r].noise.row(d)
                           * control_cost_matrix_R_ * noisy_rollouts_[r].noise.row(d).transpose());
        }
        frob_stddev =  sqrt(numer/(denom*config_.num_timesteps));

        if(!std::isnan(frob_stddev))
        {
          noise_stddevs_[d] = (1.0 - update_rate)*noise_stddevs_[d] + update_rate * frob_stddev;
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

      const std::vector<double>& min_stddev = config_.noise_generation.min_stddev;
      noise_stddevs_.resize(stddev.size());
      for(unsigned int i = 0; i < stddev.size() ; i++)
      {
        noise_stddevs_[i] = stddev[i]*pow(decay[i],config_.noise_generation.update_rate*(current_iteration_ -1));
        noise_stddevs_[i] = (noise_stddevs_[i] < min_stddev[i]) ? min_stddev[i]: noise_stddevs_[i];
      }
      break;
  }
}

bool Stomp::computeInitialTrajectory(const std::vector<double>& first,const std::vector<double>& last)
{
  // allocating
  parameters_optimized_= Eigen::MatrixXd::Zero(config_.num_dimensions,config_.num_timesteps);

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

      valid = computeMinCostTrajectory(first,last,control_cost_matrix_R_padded_,inv_control_cost_matrix_R_,parameters_optimized_);
      break;
  }

  // filtering and returning
  bool filtered;
  return task_->filterParameters(0,config_.num_timesteps,current_iteration_,parameters_optimized_,filtered);
}

bool Stomp::cancel()
{
  ROS_WARN("Interrupting STOMP");
  setProceed(false);
  return !getProceed();
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
  int rollouts_generate = config_.num_rollouts;
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
    double cost_prob;
    double weighted_prob;
    for (auto r = 0u; r<rollouts_stored; ++r)
    {

      // Apply noise generated on the previous iteration onto the current trajectory
      noisy_rollouts_[r].noise = noisy_rollouts_[r].parameters_noise
          - parameters_optimized_;

      cost_prob = exp(-h*(noisy_rollouts_[r].total_cost - min_cost)/cost_denom);
      weighted_prob = cost_prob * noisy_rollouts_[r].importance_weight;
      rollout_cost_sorter.push_back(std::make_pair(-weighted_prob,r));
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
  for(auto r = 0u; r < rollouts_generate; r++)
  {
    for (auto d = 0u; d<config_.num_dimensions; ++d)
    {
      random_dist_generators_[d]->sample(temp_noise_array_);
      noisy_rollouts_[r].noise.row(d).transpose()  = noise_stddevs_[d] * temp_noise_array_;
      noisy_rollouts_[r].parameters_noise.row(d) = parameters_optimized_.row(d) + noisy_rollouts_[r].noise.row(d);
    }

  }

  // update total active rollouts
  num_active_rollouts_ = rollouts_reuse + rollouts_generate;

  return true;
}

bool Stomp::filterNoisyRollouts()
{
  // apply post noise generation filters
  bool filtered = false;
  for(auto r = 0u ; r < config_.num_rollouts; r++)
  {
    if(!task_->filterNoisyParameters(0,config_.num_timesteps,current_iteration_,r,noisy_rollouts_[r].parameters_noise,filtered))
    {
      ROS_ERROR_STREAM("Failed to filter noisy parameters");
      return filtered;
    }

    if(filtered)
    {
      noisy_rollouts_[r].noise = noisy_rollouts_[r].parameters_noise - parameters_optimized_;
    }
  }

  return true;
}

bool Stomp::computeNoisyRolloutsCosts()
{
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
      total_control_cost = 0;
      double ccost = 0;
      for(auto d = 0u; d < config_.num_dimensions; d++)
      {
        ccost = rollout.control_costs.row(d).sum();
        total_control_cost += ccost;
        rollout.full_costs[d] = ccost + total_state_cost;
      }
      rollout.total_cost = total_state_cost + total_control_cost;

      // Compute total cost for each time step
      for(auto d = 0u; d < config_.num_dimensions; d++)
      {
        rollout.total_costs.row(d) = rollout.state_costs.transpose() + rollout.control_costs.row(d);
      }
    }
  }

  return valid;
}

bool Stomp::computeRolloutsStateCosts()
{

  bool all_valid = true;
  bool proceed = true;
  for(auto r = 0u ; r < config_.num_rollouts; r++)
  {
    if(!getProceed())
    {
      proceed = false;
      break;
    }

    Rollout& rollout = noisy_rollouts_[r];
    if(!task_->computeNoisyCosts(rollout.parameters_noise,0,
                            config_.num_timesteps,
                            current_iteration_,r,
                            rollout.state_costs,all_valid))
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
        rollout.control_costs.row(d).setConstant(0);
      }
    }
    else
    {
      computeParametersControlCosts(rollout.parameters_noise,
                                    OPTIMIZATION_TIMESTEP,
                                    config_.control_cost_weight,
                                    control_cost_matrix_R_,rollout.control_costs);
    }
  }
  return true;
}

bool Stomp::computeProbabilities()
{

  double cost;
  double min_cost;
  double max_cost;
  double denom;
  double numerator;
  double probl_sum = 0.0; // total probability sum of all rollouts for each joint
  const double h = EXPONENTIATED_COST_SENSITIVITY;
  double exponent = 0;

  for (auto d = 0u; d<config_.num_dimensions; ++d)
  {

    for (auto t = 0u; t<config_.num_timesteps; t++)
    {

      // find min and max cost over all rollouts at timestep 't':
      min_cost = noisy_rollouts_[0].total_costs(d,t);
      max_cost = min_cost;
      for (auto r=0u; r<num_active_rollouts_; ++r)
      {
          cost = noisy_rollouts_[r].total_costs(d,t);
          if (cost < min_cost)
              min_cost = cost;
          if (cost > max_cost)
              max_cost = cost;
      }

      denom = max_cost - min_cost;

      // prevent division by zero:
      if (denom < MIN_COST_DIFFERENCE)
      {
        denom = MIN_COST_DIFFERENCE;
      }

      probl_sum = 0.0;
      for (auto r = 0u; r<num_active_rollouts_; ++r)
      {
        // this is the exponential term in the probability calculation described in the literature
        exponent = -h*(noisy_rollouts_[r].total_costs(d,t) - min_cost)/denom;
        noisy_rollouts_[r].probabilities(d,t) = noisy_rollouts_[r].importance_weight *
            exp(exponent);

        probl_sum += noisy_rollouts_[r].probabilities(d,t);
      }

      // scaling each probability value by the sum of all probabilities corresponding to all rollouts at time "t"
      for (auto r = 0u; r<num_active_rollouts_; ++r)
      {
        noisy_rollouts_[r].probabilities(d,t) /= probl_sum;
      }
    }


    // computing full probabilities
    min_cost = noisy_rollouts_[0].full_costs[d];
    max_cost = min_cost;
    double c = 0.0;
    for (int r=1; r<num_active_rollouts_; ++r)
    {
      c = noisy_rollouts_[r].full_costs[d];
      if (c < min_cost)
        min_cost = c;
      if (c > max_cost)
        max_cost = c;
    }

    denom = max_cost - min_cost;
    denom = denom < 1e-8 ? 1e-8 : denom;

    probl_sum = 0.0;
    for (int r=0; r<num_active_rollouts_; ++r)
    {
      noisy_rollouts_[r].full_probabilities[d] = noisy_rollouts_[r].importance_weight *
          exp(-h*(noisy_rollouts_[r].full_costs[d] - min_cost)/denom);
      probl_sum += noisy_rollouts_[r].full_probabilities[d];
    }
    for (int r=0; r<num_active_rollouts_; ++r)
    {
        noisy_rollouts_[r].full_probabilities[d] /= probl_sum;
    }
  }

  return true;
}

bool Stomp::updateParameters()
{
  // computing updates from probabilities using convex combination
  parameters_updates_.setZero();
  for(auto d = 0u; d < config_.num_dimensions ; d++)
  {

    for(auto r = 0u; r < num_active_rollouts_; r++)
    {
      auto& rollout = noisy_rollouts_[r];
      parameters_updates_.row(d) +=  (rollout.noise.row(d).array() * rollout.probabilities.row(d).array()).matrix();
    }

  }

  // applying smoothing
  if(!task_->smoothParameterUpdates(0,config_.num_timesteps,current_iteration_,parameters_updates_))
  {
    ROS_ERROR("Update smoothing step failed");
    return false;
  }

  // updating parameters
  parameters_optimized_ += parameters_updates_;

  return true;
}

bool Stomp::filterUpdatedParameters()
{
  bool filtered = false;
  return task_->filterParameters(0,config_.num_timesteps,current_iteration_,parameters_optimized_,filtered);
}

bool Stomp::computeOptimizedCost()
{
  bool proceed = true;

  // control costs
  parameters_total_cost_ = 0;
  if(config_.control_cost_weight > MIN_CONTROL_COST_WEIGHT)
  {
    computeParametersControlCosts(parameters_optimized_,
                                  OPTIMIZATION_TIMESTEP,
                                  config_.control_cost_weight,
                                  control_cost_matrix_R_,
                                  parameters_control_costs_);

    // adding all costs
    parameters_total_cost_ = parameters_control_costs_.rowwise().sum().sum();

  }

  // state costs
  if(task_->computeCosts(parameters_optimized_,
                         0,config_.num_timesteps,current_iteration_,parameters_state_costs_,parameters_valid_))
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
