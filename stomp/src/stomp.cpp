/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

// system includes
#include <cassert>
#include <omp.h>

// ros includes
#include <ros/package.h>
#include <stomp/stomp.h>
#include <boost/filesystem.hpp>

namespace stomp
{

STOMP::STOMP()
: initialized_(false), policy_iteration_counter_(0),
  proceed_(true)
{
}

STOMP::~STOMP()
{
}

bool STOMP::initialize(const ros::NodeHandle& node_handle, boost::shared_ptr<stomp::Task> task)
{
  node_handle_ = node_handle;
  STOMP_VERIFY(readParameters());

  task_ = task;
  STOMP_VERIFY(task_->getPolicy(policy_));
  STOMP_VERIFY(policy_->getNumTimeSteps(num_time_steps_));
  control_cost_weight_ = task_->getControlCostWeight();

  STOMP_VERIFY(policy_->getNumDimensions(num_dimensions_));
  ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_decay_.size()));
  ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_stddev_.size()));
  ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_min_stddev_.size()));
  //    ROS_INFO("Learning policy with %i dimensions.", num_dimensions_);

  policy_improvement_.initialize(num_time_steps_, min_rollouts_, max_rollouts_, num_rollouts_per_iteration_,
                                 policy_, use_noise_adaptation_, noise_min_stddev_);

  rollout_costs_ = Eigen::MatrixXd::Zero(max_rollouts_, num_time_steps_);

  policy_iteration_counter_ = 0;

  // initialize openmp
  num_threads_ = omp_get_max_threads();
  if (!use_openmp_)
  {
    num_threads_ = 1;
    omp_set_num_threads(1);
  }
  //ROS_INFO("STOMP: using %d threads", num_threads_);
  tmp_rollout_cost_.resize(max_rollouts_, Eigen::VectorXd::Zero(num_time_steps_));
  tmp_rollout_weighted_features_.resize(max_rollouts_, Eigen::MatrixXd::Zero(num_time_steps_, 1));

  best_noiseless_cost_ = std::numeric_limits<double>::max();

  return (initialized_ = true);
}

bool STOMP::readParameters()
{
  STOMP_VERIFY(node_handle_.getParam("min_rollouts", min_rollouts_));
  STOMP_VERIFY(node_handle_.getParam("max_rollouts", max_rollouts_));
  STOMP_VERIFY(node_handle_.getParam("num_rollouts_per_iteration", num_rollouts_per_iteration_));
  STOMP_VERIFY(readDoubleArray(node_handle_, "noise_stddev", noise_stddev_));
  STOMP_VERIFY(readDoubleArray(node_handle_, "noise_decay", noise_decay_));
  STOMP_VERIFY(readDoubleArray(node_handle_, "noise_min_stddev", noise_min_stddev_));
  node_handle_.param("write_to_file", write_to_file_, true); // defaults are sometimes good!
  node_handle_.param("use_noise_adaptation", use_noise_adaptation_, true);
  node_handle_.param("use_openmp", use_openmp_, false);
  return true;
}

bool STOMP::readPolicy(const int iteration_number)
{
  // check whether reading the policy from file is neccessary
  if(iteration_number == (policy_iteration_counter_))
  {
    return true;
  }
  /*    ROS_INFO("Read policy from file %s.", policy_->getFileName(iteration_number).c_str());
    STOMP_VERIFY(policy_->readFromDisc(policy_->getFileName(iteration_number)));
    STOMP_VERIFY(task_->setPolicy(policy_));
   */    return true;
}

bool STOMP::writePolicy(const int iteration_number, bool is_rollout, int rollout_id)
{
  return true;
}

void STOMP::clearReusedRollouts()
{
  policy_improvement_.clearReusedRollouts();
}

bool STOMP::doGenRollouts(int iteration_number)
{
  // compute appropriate noise values
  std::vector<double> noise;
  noise.resize(num_dimensions_);
  for (int i=0; i<num_dimensions_; ++i)
  {
    noise[i] = noise_stddev_[i] * pow(noise_decay_[i], iteration_number-1);
  }

  // get rollouts
  STOMP_VERIFY(policy_improvement_.getRollouts(rollouts_, noise));
  // filter rollouts and set them back if filtered:
  bool filtered = false;
  for (unsigned int r=0; r<rollouts_.size(); ++r)
  {
    if (task_->filter(rollouts_[r], r, 0))
      filtered = true;
  }
  if (filtered)
  {
    policy_improvement_.setRollouts(rollouts_);
  }
  STOMP_VERIFY(policy_improvement_.computeProjectedNoise());

  // overwrite the rollouts with the projected versions
  policy_improvement_.getProjectedRollouts(projected_rollouts_);

  return true;
}

bool STOMP::doExecuteRollouts(int iteration_number)
{
  std::vector<Eigen::VectorXd> gradients;
#pragma omp parallel for num_threads(num_threads_)
  for (int r=0; r<int(rollouts_.size()); ++r)
  {
    int thread_id = omp_get_thread_num();
    bool validity;
    STOMP_VERIFY(task_->execute(rollouts_[r], projected_rollouts_[r], tmp_rollout_cost_[r], tmp_rollout_weighted_features_[r],
                              iteration_number, r, thread_id, false, gradients, validity));
  }
  for (int r=0; r<int(rollouts_.size()); ++r)
  {
    rollout_costs_.row(r) = tmp_rollout_cost_[r].transpose();
  }

  return true;
}

bool STOMP::doRollouts(int iteration_number)
{
  doGenRollouts(iteration_number);
  doExecuteRollouts(iteration_number);
  return true;
}

bool STOMP::doUpdate(int iteration_number)
{
  // TODO: fix this std::vector<>
  std::vector<double> all_costs;
  STOMP_VERIFY(policy_improvement_.setRolloutCosts(rollout_costs_, control_cost_weight_, all_costs));

  // improve the policy
  STOMP_VERIFY(policy_improvement_.improvePolicy(parameter_updates_));
  STOMP_VERIFY(policy_improvement_.getTimeStepWeights(time_step_weights_));
  STOMP_VERIFY(policy_->updateParameters(parameter_updates_, time_step_weights_));

  return true;
}

bool STOMP::doNoiselessRollout(int iteration_number)
{
  // get a noise-less rollout to check the cost
  std::vector<Eigen::VectorXd> gradients;
  STOMP_VERIFY(policy_->getParameters(parameters_));
  bool validity = false;
  STOMP_VERIFY(task_->execute(parameters_, parameters_, tmp_rollout_cost_[0], tmp_rollout_weighted_features_[0], iteration_number,
                            -1, 0, false, gradients, validity));
  double total_cost;
  policy_improvement_.setNoiselessRolloutCosts(tmp_rollout_cost_[0], total_cost);

  ROS_DEBUG_STREAM("Noiseless cost : "<< total_cost<<", best noiseless cost: "<<best_noiseless_cost_);

  if (total_cost < best_noiseless_cost_)
  {
    best_noiseless_parameters_ = parameters_;
    best_noiseless_cost_ = total_cost;
  }
  last_noiseless_rollout_valid_ = validity;
  return true;
}

bool STOMP::runSingleIteration(const int iteration_number)
{
  ROS_ASSERT(initialized_);
  policy_iteration_counter_++;

  if (write_to_file_)
  {
    ROS_DEBUG_STREAM(__FUNCTION__<< " reading policy from file");
    // load new policy if neccessary
    STOMP_VERIFY(readPolicy(iteration_number));
  }


  //ROS_ASSERT(doRollouts(iteration_number));
  doRollouts(iteration_number);

  //ROS_ASSERT(doUpdate(iteration_number));
  doUpdate(iteration_number);

  //ROS_ASSERT(doNoiselessRollout(iteration_number));
  doNoiselessRollout(iteration_number);

  if (write_to_file_)
  {
    // store updated policy to disc
    ROS_DEBUG_STREAM(__FUNCTION__<< " writing policy to file");
    STOMP_VERIFY(writePolicy(iteration_number));
    //STOMP_VERIFY(writePolicyImprovementStatistics(stats_msg));
  }

  return true;
}

void STOMP::getAllRollouts(std::vector<Rollout>& rollouts)
{
  policy_improvement_.getAllRollouts(rollouts);
}

void STOMP::getNoiselessRollout(Rollout& rollout)
{
  policy_improvement_.getNoiselessRollout(rollout);
}

void STOMP::getAdaptedStddevs(std::vector<double>& stddevs)
{
  policy_improvement_.getAdaptedStddevs(stddevs);
}

void STOMP::getBestNoiselessParameters(std::vector<Eigen::VectorXd>& parameters, double& cost)
{
  parameters = best_noiseless_parameters_;
  cost = best_noiseless_cost_;
}

bool STOMP::getProceed()
{
  proceed_mutex_.lock();
  bool r = proceed_;
  proceed_mutex_.unlock();
  return r;

}

void STOMP::proceed(bool proceed)
{
  proceed_mutex_.lock();
  proceed_ = proceed;
  proceed_mutex_.unlock();

}

bool STOMP::runUntilValid(int max_iterations, int iterations_after_collision_free)
{
  int collision_free_iterations = 0;
  unsigned int num_iterations = 0;
  bool success = false;
  proceed(true);
  for (int i=0; i<max_iterations; ++i)
  {
    if(!getProceed())
    {
      ROS_DEBUG_STREAM("STOMP was interrupted");
      success = false;
      break;
    }

    runSingleIteration(i);
    task_->onEveryIteration();
    if (last_noiseless_rollout_valid_)
    {
      success = true;
      collision_free_iterations++;
    }

    if (collision_free_iterations>=iterations_after_collision_free)
    {
      break;
    }

    num_iterations++;
  }


  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__<< " completed with success = "<<success<<" after "<<num_iterations<<" iterations");

  return success;
}

void STOMP::setCostCumulation(bool use_cumulative_costs)
{
  policy_improvement_.setCostCumulation(use_cumulative_costs);
}

void STOMP::resetAdaptiveNoise()
{
  policy_improvement_.resetAdaptiveNoise();
}

}
