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
#include "stomp_core/stomp.h"
#include "stomp_core/stomp_core_utils.h"

static const double DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT = 1.0;
static const double EXPONENTIATED_COST_SENSITIVITY = 10;

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


namespace stomp_core {

Stomp::Stomp(const StompConfiguration& config,TaskPtr task):
    config_(config),
    task_(task),
    proceed_(true),
    optimized_parameters_total_cost_(0),
    optimized_parameters_valid_(false),
    num_active_rollouts_(0),
    current_iteration_(0)
{

}

Stomp::~Stomp()
{

}

bool Stomp::solve(const std::vector<double>& first,const std::vector<double>& last,
                  std::vector<Eigen::VectorXd>& optimized_parameters)
{
  // initialize trajectory
  if(!computeInitialTrajectory(first,last))
  {
    ROS_ERROR("Unable to generate initial trajectory");
  }

  return solve(optimized_parameters_,optimized_parameters);
}

bool Stomp::solve(const std::vector<Eigen::VectorXd>& initial_parameters,
                  std::vector<Eigen::VectorXd>& optimized_parameters)
{

  // check initial trajectory size
  if(initial_parameters.size() != config_.dimensions)
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

  if(!initializeOptimizationVariables())
  {
    return false;
  }

  setProceed(true);


  current_iteration_ = 0;
  unsigned int valid_iterations = 0;
  double lowest_cost = std::numeric_limits<double>::max();
  while(current_iteration_ < config_.num_iterations && runSingleIteration())
  {

    if(optimized_parameters_valid_)
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

    if(optimized_parameters_total_cost_ < lowest_cost)
    {
      lowest_cost = optimized_parameters_total_cost_;
    }

    current_iteration_++;
  }

  if(optimized_parameters_valid_)
  {
    ROS_INFO("Stomp found a valid solution with cost %f after %i iterations",
             lowest_cost,current_iteration_);
  }
  else
  {
    ROS_ERROR("Stomp failed to find a valid solution after %i iterations",current_iteration_);
  }

  return optimized_parameters_valid_;
}

bool Stomp::initializeOptimizationVariables()
{
  // generate finite difference matrix
  generateFiniteDifferenceMatrix(config_.num_timesteps,DerivativeOrders::STOMP_ACCELERATION,
                                 config_.delta_t,acc_diff_matrix_);

  // control cost matrix
  control_cost_matrix_ = acc_diff_matrix_.transpose() * acc_diff_matrix_;
  smooth_update_matrix_ = control_cost_matrix_.fullPivLu().inverse();

  // noise generation
  mv_gaussian_.reset(new MultivariateGaussian(Eigen::VectorXd::Zero(config_.num_timesteps),smooth_update_matrix_));
  noise_stddevs_ = config_.noise_generation.stddev;

  // noisy rollouts allocation
  int d = config_.dimensions;
  num_active_rollouts_ = 0;
  noisy_rollouts_.resize(config_.max_rollouts);

  // initializing rollout
  Rollout rollout;
  rollout.noise.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.parameters_noise.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));

  rollout.probabilities.resize(d,Eigen::VectorXd::Zero(config_.num_timesteps));
  rollout.full_probabilities.resize(config_.num_timesteps);

  rollout.full_costs.resize(config_.num_timesteps);
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
      const std::vector<double>& init_stddev = config_.noise_generation.stddev;
      const std::vector<double>& min_stddev = config_.noise_generation.min_stddev;
      for(unsigned int d = 0; d < config_.dimensions; d++)
      {
        denom = 0;
        numer = 0;
        for (int r=0; r<num_active_rollouts_; ++r)
        {
            denom += noisy_rollouts_[r].full_probabilities[d];
            numer += noisy_rollouts_[r].full_probabilities[d] *
                    double(noisy_rollouts_[r].noise[d].transpose() * control_cost_matrix_ * noisy_rollouts_[r].noise[d]);
        }
        frob_stddev = sqrt(numer/(denom*config_.num_timesteps));
        frob_stddev = std::isnan(frob_stddev) ? 0 : frob_stddev;

        if(std::isnan(frob_stddev))
        {
          noise_stddevs_[d] = init_stddev[d];
        }
        else
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
  optimized_parameters_.resize(config_.dimensions,Eigen::VectorXd::Zero(config_.num_timesteps));
  bool valid = true;

  switch(config_.initialization_method)
  {
    case TrajectoryInitializations::CUBIC_POLYNOMIAL_INTERPOLATION:

      computeCubicInterpolation(first,last,config_.num_timesteps,config_.delta_t,optimized_parameters_);
      break;
    case TrajectoryInitializations::LINEAR_INTERPOLATION:

      computeLinearInterpolation(first,last,config_.num_timesteps,optimized_parameters_);
      break;
    case TrajectoryInitializations::MININUM_CONTROL_COST:

      valid = computeMinCostTrajectory(first,last,smooth_update_matrix_,optimized_parameters_);
      break;
  }

  return valid;
}

bool Stomp::cancel()
{
  ROS_WARN("Interrupting Stomp");
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

  bool succeed = getProceed() &&
      generateNoisyRollouts() &&
      computeRolloutsCosts() &&
      computeProbabilities() &&
      updateParameters() &&
      computeOptimizedCost();

  return succeed;
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
     for (int r=0; r<rollouts_stored; ++r)
     {

       // Apply noise generated on the previous iteration onto the current trajectory
       for (int d=0; d<config_.dimensions; ++d)
        {
          noisy_rollouts_[r].parameters_noise[d] = optimized_parameters_[d] + noisy_rollouts_[r].noise[d];
        }

       double cost_prob = exp(-h*(noisy_rollouts_[r].total_cost - min_cost)/cost_denom);
       double weighted_cost = cost_prob * noisy_rollouts_[r].importance_weight;
       rollout_cost_sorter.push_back(std::make_pair(-weighted_cost,r));
     }

     std::sort(rollout_cost_sorter.begin(), rollout_cost_sorter.end());

     // use the best ones: (copy them into reused_rollouts)
     for (int r=0; r<rollouts_stored; ++r)
     {
       int reuse_index = rollout_cost_sorter[r].second;
       reused_rollouts_[r] = noisy_rollouts_[reuse_index];
     }

     // copy them back from reused_rollouts_ into rollouts_
     for (int r=0; r<rollouts_reuse; ++r)
     {
       noisy_rollouts_[rollouts_generate+r] = reused_rollouts_[r];
     }
  }

  // generate new rollouts


}

bool Stomp::computeRolloutsCosts()
{

}

bool Stomp::computeProbabilities()
{

}

bool Stomp::updateParameters()
{

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
