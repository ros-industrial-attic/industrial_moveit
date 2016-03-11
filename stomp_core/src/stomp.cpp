/*
 * stomp.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Jorge Nicho
 */

#include <ros/console.h>
#include <limits.h>
#include "stomp_core/stomp.h"
#include "stomp_core/stomp_core_utils.h"

static void computeLinearInterpolation(const std::vector<double>& first,const std::vector<double>& last,
                         int num_timesteps,
                         std::vector<Eigen::VectorXf>& trajectory_joints)
{
  trajectory_joints.resize(first.size(),Eigen::VectorXf::Zero(num_timesteps));
  for(int unsigned i = 0; i < first.size(); i++)
  {
    double dtheta = (last[i] - first[i])/(num_timesteps - 1);
    for(unsigned int j = 0; j < num_timesteps; j++)
    {
      trajectory_[i](j) = first[d] + j * dtheta;
    }
  }
}

static void computeCubicInterpolation(const std::vector<double>& first,const std::vector<double>& last,
                         int num_points,double dt,
                         std::vector<Eigen::VectorXf>& trajectory_joints)
{
  std::vector<double> coeffs(4,0);
  double total_time = (num_points - 1) * dt;
  for(int unsigned i = 0; i < first.size(); i++)
  {
    coeffs[0] = first[i];
    coeffs[2] = (3/(pow(total_time,2))) * (last[i] - first[i]);
    coeffs[3] = (3-2(pow(total_time,3))) * (last[i] - first[i]);

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
                              const Eigen::MatrixXd& smoothing_matrix,
                              std::vector<Eigen::VectorXf>& trajectory_joints)
{
  using namespace stomp_core;

  if(smoothing_matrix.rows() != smoothing_matrix.cols())
  {
    ROS_ERROR("Smoothing Matrix is not square");
    return false;
  }

  int timesteps = smoothing_matrix.rows();
  Eigen::VectorXd linear_control_cost = Eigen::VectorXd::Zero(timesteps);
  trajectory_joints.resize(first.size(),Eigen::VectorXf::Zero(timesteps));

  for(unsigned int i = 0; i < first.size(); i++)
  {
    linear_control_cost.transpose() = first[i] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        smooth_update_matrix_.block(0,0,FINITE_DIFF_RULE_LENGTH - 1,timesteps);

    linear_control_cost.transpose() += last[i] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        smooth_update_matrix_.block(timesteps - FINITE_DIFF_RULE_LENGTH + 1,0,FINITE_DIFF_RULE_LENGTH - 1,
                                    timesteps);

    linear_control_cost *=2;

    trajectory_joints[i] = -0.5*smoothing_matrix*linear_control_cost;

  }

  return true;
}


namespace stomp_core {

Stomp::Stomp(const StompConfiguration& config,TaskPtr task):
    config_(config),
    task_(taks),
    proceed_(true),
    optimized_parameters_total_cost_(0),
    optimized_parameters_valid_(false),
    num_active_rollouts_(0)
{

}

Stomp::~Stomp()
{

}

bool Stomp::solve(const std::vector<double>& first,const std::vector<double>& last,
                  std::vector<Eigen::VectorXf>& optimized_parameters)
{

}

bool Stomp::solve(const std::vector<Eigen::VectorXf>& initial_parameters,
                  std::vector<Eigen::VectorXf>& optimized_parameters)
{

  setProceed(true);

  unsigned int iterations = 0;
  unsigned int valid_iterations = 0;
  double lowest_cost = std::numeric_limits<double>::max();
  while(iterations < config_.num_iterations && runSingleIteration())
  {

    if(optimized_parameters_valid_)
    {
      valid_iterations++;
      ROS_DEBUG("Found valid solution, will iterate %i more times ",
               config_.iterations_after_valid - valid_iterations);
    }
    else
    {
      valid_iterations = 0;
    }

    if(valid_iterations>= config_.iterations_after_valid)
    {
      break;
    }

    if(optimized_parameters_total_cost_ < lowest_cost)
    {
      lowest_cost = optimized_parameters_total_cost_;
    }

    iterations++;
  }

  if(optimized_parameters_valid_)
  {
    ROS_INFO("Stomp found a valid solution with cost %f after %i iterations",
             lowest_cost,iterations);
  }
  else
  {
    ROS_ERROR("Stomp failed to find a valid solution after %i iterations",iterations);
  }

  return optimized_parameters_valid_;
}

bool Stomp::initializeOptimizationMatrices()
{
  // generate finite difference matrix
  generateFiniteDifferenceMatrix(config_.timesteps,DerivativeOrders::STOMP_ACCELERATION,
                                 config_.delta_t,acc_diff_matrix_);

  // control cost matrix
  control_cost_matrix_ = acc_diff_matrix_.transpose() * acc_diff_matrix_;
  smooth_update_matrix_ = control_cost_matrix_.inverse();

}

bool Stomp::computeInitialTrajectory(const std::vector<double>& first,const std::vector<double>& last)
{
  // allocating
  optimized_parameters_.resize(config_.dimensions,Eigen::VectorXf::Zero(config_.num_timesteps));
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

      valid = computeMinCostTrajectory(first,last,smooth_update_matrix_,optimized_parameters_)
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
  proceed_mutex_.lock();D size array of
  proceed = proceed_;
  proceed_mutex_.unlock();
  return proceed;
}


} /* namespace stomp */
