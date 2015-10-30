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

 \file    policy_improvement.cpp

 \author  Ludovic Righetti, Peter Pastor, Mrinal Kalakrishnan
 \date    May 26, 2010

 **********************************************************************/

// system includes
#include <time.h>
#include <cfloat>
#include <ros/assert.h>

#include <Eigen/Core>
#include <Eigen/LU>

// local includes
#include <ros/ros.h>
#include <stomp/policy_improvement.h>
#include <algorithm>

using namespace Eigen;

namespace stomp
{

PolicyImprovement::PolicyImprovement():
    initialized_(false)
{
  cost_scaling_h_ = 10.0;
  use_cumulative_costs_ = true;
  use_projection_ = false;
}

PolicyImprovement::~PolicyImprovement()
{
}

bool PolicyImprovement::initialize(const int num_time_steps,
                                   const int min_rollouts,
                                   const int max_rollouts,
                                   const int num_rollouts_per_iteration,
                                   boost::shared_ptr<CovariantMovementPrimitive> policy,
                                   double control_cost_weight,
                                   bool use_noise_adaptation,
                                   const std::vector<double>& noise_min_stddev)
{
  num_time_steps_ = num_time_steps;
  noise_min_stddev_ = noise_min_stddev;
  policy_ = policy;
  use_covariance_matrix_adaptation_ = use_noise_adaptation;
  adapted_covariance_valid_ = false;
  control_cost_weight_ = control_cost_weight;

  STOMP_VERIFY(policy_->setNumTimeSteps(num_time_steps_));
  STOMP_VERIFY(policy_->getControlCosts(control_costs_));
  STOMP_VERIFY(policy_->getNumDimensions(num_dimensions_));
  STOMP_VERIFY(policy_->getNumParameters(num_parameters_));
  STOMP_VERIFY(policy_->getBasisFunctions(basis_functions_));
  STOMP_VERIFY(policy_->getParameters(parameters_));
  STOMP_VERIFY(policy_->getInvControlCosts(inv_control_costs_));

  // invert the control costs, initialize noise generators:
  noise_generators_.clear();
  adapted_stddevs_.resize(num_dimensions_, 1.0);
  adapted_covariances_.clear();
  for (int d=0; d<num_dimensions_; ++d)
  {
    MultivariateGaussian mvg(VectorXd::Zero(num_parameters_[d]), inv_control_costs_[d]);
    noise_generators_.push_back(mvg);
    adapted_covariances_.push_back(inv_control_costs_[d]);
  }

  noiseless_rollout_valid_ = false;

  STOMP_VERIFY(setNumRollouts(min_rollouts, max_rollouts, num_rollouts_per_iteration));
  STOMP_VERIFY(preAllocateTempVariables());
  STOMP_VERIFY(preComputeProjectionMatrices());

  return (initialized_ = true);
}

bool PolicyImprovement::setNumRollouts(const int min_rollouts,
                                       const int max_rollouts,
                                       const int num_rollouts_per_iteration)
{
  min_rollouts_ = min_rollouts;
  max_rollouts_ = max_rollouts;
  num_rollouts_per_iteration_ = num_rollouts_per_iteration;
  num_rollouts_ = 0;
  num_rollouts_gen_ = 0;

  // preallocate memory for a single rollout:
  Rollout rollout;

  rollout.parameters_.clear();
  rollout.parameters_noise_.clear();
  rollout.noise_.clear();
  rollout.noise_projected_.clear();
  rollout.parameters_noise_projected_.clear();
  rollout.control_costs_.clear();
  rollout.total_costs_.clear();
  rollout.cumulative_costs_.clear();
  rollout.probabilities_.clear();
  rollout.full_probabilities_.resize(num_dimensions_);
  rollout.full_costs_.resize(num_dimensions_);
  for (int d=0; d<num_dimensions_; ++d)
  {
      rollout.parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
      rollout.parameters_noise_.push_back(VectorXd::Zero(num_parameters_[d]));
      rollout.parameters_noise_projected_.push_back(VectorXd::Zero(num_parameters_[d]));
      rollout.noise_.push_back(VectorXd::Zero(num_parameters_[d]));
      rollout.noise_projected_.push_back(VectorXd::Zero(num_parameters_[d]));
      rollout.control_costs_.push_back(VectorXd::Zero(num_time_steps_));
      rollout.total_costs_.push_back(VectorXd::Zero(num_time_steps_));
      rollout.cumulative_costs_.push_back(VectorXd::Zero(num_time_steps_));
      rollout.probabilities_.push_back(VectorXd::Zero(num_time_steps_));
  }
  rollout.state_costs_ = VectorXd::Zero(num_time_steps_);

  // duplicate this rollout:
  for (int r=0; r<max_rollouts_+1; ++r)
  {
    rollouts_.push_back(rollout);
    reused_rollouts_.push_back(rollout);
  }
  noiseless_rollout_ = rollout;

  rollout_cost_sorter_.reserve(max_rollouts_);

  return true;
}

bool PolicyImprovement::generateRollouts(const std::vector<double>& noise_stddev)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(static_cast<int>(noise_stddev.size()) == num_dimensions_);
  if (!adapted_covariance_valid_)
    adapted_stddevs_ = noise_stddev;

  // save the latest policy parameters:
  STOMP_VERIFY(copyParametersFromPolicy());


  // decide how many new rollouts we will generate and discard
  int num_rollouts_discard = 0;
  int num_rollouts_reused = num_rollouts_;
  int prev_num_rollouts = num_rollouts_;
  num_rollouts_gen_ = num_rollouts_per_iteration_;
  if (num_rollouts_ + num_rollouts_gen_ < min_rollouts_)
  {
    num_rollouts_gen_ = min_rollouts_ - num_rollouts_;
    num_rollouts_discard = 0;
    num_rollouts_reused = num_rollouts_;
  }

  if (num_rollouts_ + num_rollouts_gen_ > max_rollouts_)
  {
    num_rollouts_discard = num_rollouts_ + num_rollouts_gen_ - max_rollouts_;
    num_rollouts_reused = num_rollouts_ - num_rollouts_discard;
  }
  num_rollouts_ = num_rollouts_reused + num_rollouts_gen_;

  if (num_rollouts_reused > 0)
  {
    // find min and max cost for exponential cost scaling
    double min_cost = rollouts_[0].total_cost_;
    double max_cost = min_cost;
    for (int r=1; r<prev_num_rollouts; ++r)
    {
      double c = rollouts_[r].total_cost_;
      if (c < min_cost)
        min_cost = c;
      if (c > max_cost)
        max_cost = c;
    }

    double cost_denom = max_cost - min_cost;
    if (cost_denom < 1e-8)
      cost_denom = 1e-8;

    // compute importance weights for all rollouts
    rollout_cost_sorter_.clear();
    for (int r=0; r<prev_num_rollouts; ++r)
    {
      // update the noise based on the new parameters:
      rollouts_[r].parameters_ = parameters_;
      double new_log_likelihood = 0.0;
      for (int d=0; d<num_dimensions_; ++d)
      {
        // parameters_noise_projected remains the same, compute everything else from it.
        rollouts_[r].noise_projected_[d] = rollouts_[r].parameters_noise_projected_[d] - parameters_[d];
        rollouts_[r].noise_[d] = inv_projection_matrix_[d] * rollouts_[r].noise_projected_[d];
        // TODO FIXME BLAH
        //rollouts_[r].noise_[d] = rollouts_[r].noise_projected_[d];
        rollouts_[r].parameters_noise_[d] = parameters_[d] + rollouts_[r].noise_[d];

//        new_log_likelihood +=  -num_time_steps_*log(adapted_stddevs_[d])
//                          -(0.5/(adapted_stddevs_[d]*adapted_stddevs_[d])) * rollouts_[r].noise_[d].transpose()
//                          * control_costs_[d] * rollouts_[r].noise_[d];

      }
//      rollouts_[r].importance_weight_ *= exp((new_log_likelihood - rollouts_[r].log_likelihood_)
//                                             /(num_dimensions_*num_time_steps_));
      rollouts_[r].importance_weight_ = 1.0;
      rollouts_[r].log_likelihood_ = new_log_likelihood;

      double cost_prob = exp(-cost_scaling_h_*(rollouts_[r].total_cost_ - min_cost)/cost_denom);
      double weighted_cost = cost_prob * rollouts_[r].importance_weight_;
      rollout_cost_sorter_.push_back(std::make_pair(-weighted_cost,r));
    }

    std::sort(rollout_cost_sorter_.begin(), rollout_cost_sorter_.end());

    // use the best ones: (copy them into reused_rollouts)
    for (int r=0; r<num_rollouts_reused; ++r)
    {
      int reuse_index = rollout_cost_sorter_[r].second;
      reused_rollouts_[r] = rollouts_[reuse_index];
      //double reuse_cost = rollout_cost_sorter_[r].first;
    }

    // copy them back from reused_rollouts_ into rollouts_
    for (int r=0; r<num_rollouts_reused; ++r)
    {
      rollouts_[num_rollouts_gen_+r] = reused_rollouts_[r];

//      ROS_INFO("Reuse %d, cost = %lf, weight=%lf",
//               r, rollouts_[num_rollouts_gen_+r].total_cost_,
//               rollouts_[num_rollouts_gen_+r].importance_weight_);
    }
  }

  // generate new rollouts
  for (int d=0; d<num_dimensions_; ++d)
  {
    // compute parameters for mean-shifted sampling
    // variance = (var_1 + var_2)^-1
    double l1 = control_cost_weight_;
    double l2 = 1.0/(adapted_stddevs_[d]*adapted_stddevs_[d]);
    double new_stddev = 1.0/sqrt(l1 + l2);


    double p1 = l1 / (l1 + l2);
    double p2 = l2 / (l1 + l2);
    //ROS_DEBUG("d = %d, l1 = %f, l2 = %f, p1 = %f, p2 = %f", d, l1, l2, p1, p2);

    for (int r=0; r<num_rollouts_gen_; ++r)
    {
      noise_generators_[d].sample(tmp_noise_[d]);

      rollouts_[r].parameters_noise_[d] =
          p1 * policy_->getMinControlCostParameters()[d] + p2 * parameters_[d] // the mean
          + new_stddev * tmp_noise_[d]; // the noise

      //rollouts_[r].noise_[d] = new_stddev*tmp_noise_[d];
      rollouts_[r].parameters_[d] = parameters_[d];// + rollouts_[r].noise_[d];
      rollouts_[r].noise_[d] = rollouts_[r].parameters_noise_[d] - rollouts_[r].parameters_[d];
      //rollouts_[r].parameters_noise_[d] = parameters_[d] + rollouts_[r].noise_[d];
    }
  }

  // compute likelihoods of new rollouts
  for (int r=0; r<num_rollouts_gen_; ++r)
  {
//    rollouts_[r].log_likelihood_ = 0.0;
//    for (int d=0; d<num_dimensions_; ++d)
//    {
//      rollouts_[r].log_likelihood_ += -num_time_steps_*log(adapted_stddevs_[d])
//                                      -(0.5/(adapted_stddevs_[d]*adapted_stddevs_[d]))
//                                      * rollouts_[r].noise_[d].transpose() * control_costs_[d]
//                                      * rollouts_[r].noise_[d];
//    }
    rollouts_[r].importance_weight_ = 1.0;
    //ROS_INFO("New rollout ln lik = %lf", rollouts_[r].log_likelihood_);
  }

  // add the noiseless rollout if it exists:
  if (noiseless_rollout_valid_)
  {
    rollouts_[num_rollouts_] = noiseless_rollout_;
    ++num_rollouts_;
  }

  return true;
}

bool PolicyImprovement::getRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, const std::vector<double>& noise_variance)
{
    if (!generateRollouts(noise_variance))
    {
        ROS_ERROR("Failed to generate rollouts.");
        return false;
    }

    rollouts.clear();
    for (int r=0; r<num_rollouts_gen_; ++r)
    {
        rollouts.push_back(rollouts_[r].parameters_noise_);
    }

    return true;
}

bool PolicyImprovement::getProjectedRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts)
{
  rollouts.clear();
  for (int r=0; r<num_rollouts_gen_; ++r)
  {
      rollouts.push_back(rollouts_[r].parameters_noise_projected_);
  }

  return true;
}


bool PolicyImprovement::setRollouts(const std::vector<std::vector<Eigen::VectorXd> >& rollouts)
{
  ROS_ASSERT((int)rollouts.size() == num_rollouts_gen_);
  for (int r=0; r<num_rollouts_gen_; ++r)
  {
    rollouts_[r].parameters_noise_ = rollouts[r];
    computeNoise(rollouts_[r]);
  }
  return true;
}

void PolicyImprovement::clearReusedRollouts()
{
  num_rollouts_ = 0;
}

bool PolicyImprovement::setRolloutCosts(const Eigen::MatrixXd& costs, const double control_cost_weight, std::vector<double>& rollout_costs_total)
{
  ROS_ASSERT(initialized_);

  control_cost_weight_ = control_cost_weight;
  computeRolloutControlCosts();

  for (int r=0; r<num_rollouts_gen_; ++r)
  {
    rollouts_[r].state_costs_ = costs.row(r).transpose();
  }

  computeRolloutCumulativeCosts(rollout_costs_total);

  //debug
//  for (int r=0; r<num_rollouts_gen_; ++r)
//  {
//    //ROS_INFO("Noisy %d, cost = %lf", r, rollouts_[r].total_cost_);
//  }
  return true;
}

//bool PolicyImprovement::addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rollout_costs)
//{
//    ROS_ASSERT(int(rollouts.size()) == num_rollouts_extra_);
//
//    // update our parameter values, so that the computed noise is correct:
//    STOMP_VERIFY(copyParametersFromPolicy());
//
//    for (int r=0; r<num_rollouts_extra_; ++r)
//    {
//        extra_rollouts_[r].parameters_ = rollouts[r];
//        extra_rollouts_[r].state_costs_ = rollout_costs[r];
//        computeNoise(extra_rollouts_[r]);
//        computeProjectedNoise(extra_rollouts_[r]);
//        computeRolloutControlCosts(extra_rollouts_[r]);
//        //ROS_INFO("Extra rollout cost = %f", extra_rollouts_[r].getCost());
//    }
//
//    extra_rollouts_added_ = true;
//    return true;
//}

bool PolicyImprovement::setNoiselessRolloutCosts(const Eigen::VectorXd& costs, double& total_cost)
{
  policy_->getParameters(noiseless_rollout_.parameters_);
  for (int d=0; d<num_dimensions_; ++d)
  {
    noiseless_rollout_.noise_[d] = Eigen::VectorXd::Zero(num_parameters_[d]);
    noiseless_rollout_.noise_projected_[d] = Eigen::VectorXd::Zero(num_parameters_[d]);
    noiseless_rollout_.parameters_noise_[d] = noiseless_rollout_.parameters_[d];
    noiseless_rollout_.parameters_noise_projected_[d] = noiseless_rollout_.parameters_[d];
  }
  noiseless_rollout_.state_costs_ = costs;
  noiseless_rollout_.importance_weight_ = 1.0;
  computeRolloutControlCosts(noiseless_rollout_);
  //std::cout << "Noiseless control costs: \n" <<  noiseless_rollout_.control_costs_[0] << "\n";
  computeRolloutCumulativeCosts(noiseless_rollout_);
  total_cost = noiseless_rollout_.total_cost_;
  noiseless_rollout_valid_ = true;
  return true;
}

bool PolicyImprovement::computeProjectedNoise()
{
    for (int r=0; r<num_rollouts_; ++r)
    {
        computeProjectedNoise(rollouts_[r]);
    }
    return true;
}

bool PolicyImprovement::computeProjectedNoise(Rollout& rollout)
{
  //ros::WallTime start_time = ros::WallTime::now();
  for (int d=0; d<num_dimensions_; ++d)
  {
    rollout.noise_projected_[d] = projection_matrix_[d] * rollout.noise_[d];
    rollout.parameters_noise_projected_[d] = rollout.parameters_[d] + rollout.noise_projected_[d];
  }
  //ROS_INFO("Noise projection took %f seconds", (ros::WallTime::now() - start_time).toSec());
  return true;
}

bool PolicyImprovement::computeRolloutControlCosts()
{
    for (int r=0; r<num_rollouts_; ++r)
    {
        computeRolloutControlCosts(rollouts_[r]);
    }
    return true;
}

bool PolicyImprovement::computeRolloutCumulativeCosts(Rollout& rollout)
{
  // set the total costs
  double state_cost = rollout.state_costs_.sum();
  double cost = state_cost;
  for (int d=0; d<num_dimensions_; ++d)
  {
    double cc_sum = rollout.control_costs_[d].sum();
    rollout.full_costs_[d] = state_cost + cc_sum;
    cost += cc_sum;
  }
  rollout.total_cost_ = cost;

  // compute cumulative costs at each timestep
  for (int d=0; d<num_dimensions_; ++d)
  {
      rollout.total_costs_[d] = rollout.state_costs_ + rollout.control_costs_[d];
      rollout.cumulative_costs_[d] = rollout.total_costs_[d];
      if (use_cumulative_costs_)
      {

        // this is forward cumulation
//          for (int t=num_time_steps_-2; t>=0; --t)
//          {
//              rollout.cumulative_costs_[d](t) += rollout.cumulative_costs_[d](t+1);
//          }

        // this is total cumulation
        rollout.cumulative_costs_[d] = Eigen::VectorXd::Ones(num_time_steps_) * rollout.total_costs_[d].sum();
      }
  }
  return true;
}

bool PolicyImprovement::computeRolloutCumulativeCosts(std::vector<double>& rollout_costs_total)
{
    rollout_costs_total.resize(num_rollouts_);
    for (int r=0; r<num_rollouts_; ++r)
    {
      computeRolloutCumulativeCosts(rollouts_[r]);
      rollout_costs_total[r] = rollouts_[r].total_cost_;
    }
    return true;
}

bool PolicyImprovement::computeRolloutProbabilities()
{
    for (int d=0; d<num_dimensions_; ++d)
    {
      // find min and max cost over all rollouts:
      double min_cost = rollouts_[0].cumulative_costs_[d].minCoeff();
      double max_cost = rollouts_[0].cumulative_costs_[d].maxCoeff();
      for (int r=1; r<num_rollouts_; ++r)
      {
        double min_r = rollouts_[r].cumulative_costs_[d].minCoeff();
        double max_r = rollouts_[r].cumulative_costs_[d].maxCoeff();
        if (min_cost > min_r)
          min_cost = min_r;
        if (max_cost < max_r)
          max_cost = max_r;
      }

        for (int t=0; t<num_time_steps_; t++)
        {

//            // find min and max cost over all rollouts:
//            double min_cost = rollouts_[0].cumulative_costs_[d](t);
//            double max_cost = min_cost;
//            for (int r=1; r<num_rollouts_; ++r)
//            {
//                double c = rollouts_[r].cumulative_costs_[d](t);
//                if (c < min_cost)
//                    min_cost = c;
//                if (c > max_cost)
//                    max_cost = c;
//            }

            double denom = max_cost - min_cost;

            //time_step_weights_[d][t] = denom;
            time_step_weights_[d][t] = 1.0;

            // prevent divide by zero:
            if (denom < 1e-8)
                denom = 1e-8;

            double p_sum = 0.0;
            for (int r=0; r<num_rollouts_; ++r)
            {
                rollouts_[r].probabilities_[d](t) = rollouts_[r].importance_weight_ *
                    exp(-cost_scaling_h_*(rollouts_[r].cumulative_costs_[d](t) - min_cost)/denom);
                p_sum += rollouts_[r].probabilities_[d](t);
            }
            for (int r=0; r<num_rollouts_; ++r)
            {
                rollouts_[r].probabilities_[d](t) /= p_sum;
            }

        }

        // now the "total" probabilities

        min_cost = rollouts_[0].full_costs_[d];
        max_cost = min_cost;
        for (int r=1; r<num_rollouts_; ++r)
        {
          double c = rollouts_[r].full_costs_[d];
          if (c < min_cost)
            min_cost = c;
          if (c > max_cost)
            max_cost = c;
        }
        double cost_denom = max_cost - min_cost;
        if (cost_denom < 1e-8)
          cost_denom = 1e-8;

        double p_sum = 0.0;
        for (int r=0; r<num_rollouts_; ++r)
        {
          rollouts_[r].full_probabilities_[d] = rollouts_[r].importance_weight_ * exp(-cost_scaling_h_*(rollouts_[r].full_costs_[d] - min_cost)/cost_denom);
          p_sum += rollouts_[r].full_probabilities_[d];
        }
        for (int r=0; r<num_rollouts_; ++r)
        {
            rollouts_[r].full_probabilities_[d] /= p_sum;
        }

    }
    return true;
}

bool PolicyImprovement::computeParameterUpdates()
{
  for (int d=0; d<num_dimensions_; ++d)
  {
    parameter_updates_[d] = MatrixXd::Zero(num_time_steps_, num_parameters_[d]);

    for (int r=0; r<num_rollouts_; ++r)
    {
      parameter_updates_[d].row(0).transpose() +=
          (rollouts_[r].noise_[d].array() * rollouts_[r].probabilities_[d].array()).matrix();
    }

    if (use_covariance_matrix_adaptation_)
    {

      // true CMA method
//      adapted_covariances_[d] = Eigen::MatrixXd::Zero(num_time_steps_, num_time_steps_);
//      for (int r=0; r<num_rollouts_; ++r)
//      {
//        adapted_covariances_[d] += rollouts_[r].full_probabilities_[d] *
//            rollouts_[r].noise_[d] * rollouts_[r].noise_[d].transpose();
//      }
//
//      ROS_INFO_STREAM("Covariance for dimension " << d << " = " << adapted_covariances_[d]);
//      adapted_stddevs_[d] = 1.0;
//      adapted_covariance_inverse_[d] = adapted_covariances_[d].fullPivLu().inverse();
//      noise_generators_[d] = MultivariateGaussian(VectorXd::Zero(num_parameters_[d]), adapted_covariances_[d]);

      // one-dimensional CMA-ish
//      double var = 0.0;
//      for (int r=0; r<num_rollouts_; ++r)
//      {
//        double dist = rollouts_[r].noise_[d].transpose() *
//            adapted_covariance_inverse_[d] * rollouts_[r].noise_[d];
//        var += rollouts_[r].full_probabilities_[d] * dist;
//        //printf("Rollout %d, dist = %f", r, dist);
//      }
//      var /= num_time_steps_;
//      adapted_stddevs_[d] = 0.8 * adapted_stddevs_[d] + 0.2 * sqrt(var);
//      ROS_INFO("Dimension %d: new stddev = %f", d, adapted_stddevs_[d]);

      double frob_stddev = 0.0, numer=0.0, denom=0.0;

      /*
      // true CMA method + minimization of frobenius norm
      adapted_covariances_[d] = Eigen::MatrixXd::Zero(num_time_steps_, num_time_steps_);
      for (int r=0; r<num_rollouts_; ++r)
      {
        adapted_covariances_[d] += rollouts_[r].full_probabilities_[d] *
            rollouts_[r].noise_[d] * rollouts_[r].noise_[d].transpose();
      }

      // minimize frobenius norm of diff between a_c and std_dev^2 * inv_control_cost
      numer = 0.0;
      denom = 0.0;
      for (int i=0; i<num_time_steps_; ++i)
      {
        for (int j=0; j<num_time_steps_; ++j)
        {
          numer += adapted_covariances_[d](i,j) * inv_control_costs_[d](i,j);
          denom += inv_control_costs_[d](i,j) * inv_control_costs_[d](i,j);
        }
      }
      frob_stddev = sqrt(numer/denom);
      double prev_frob_stddev = frob_stddev;
*/

      // new KL divergence M projection / ML estimation method
      denom = 0.0;
      numer = 0.0;
      for (int r=0; r<num_rollouts_; ++r)
      {
    	  denom += rollouts_[r].full_probabilities_[d];
    	  numer += rollouts_[r].full_probabilities_[d] *
    			  double(rollouts_[r].noise_[d].transpose() * control_costs_[d] * rollouts_[r].noise_[d]);
      }
      frob_stddev = sqrt(numer/(denom*num_time_steps_));
      frob_stddev = std::isnan(frob_stddev) ? 1 : frob_stddev;

      double update_rate = 0.2;
      adapted_stddevs_[d] = (1.0 - update_rate) * adapted_stddevs_[d] + update_rate * frob_stddev;

      if (adapted_stddevs_[d] < noise_min_stddev_[d])
        adapted_stddevs_[d] = noise_min_stddev_[d];
//      ROS_INFO("Dimension %d: new stddev = %f", d, adapted_stddevs_[d]);

      adapted_covariance_valid_ = true;

    }

    // reweighting the updates per time-step
    double weight = 0.0;
    double weight_sum = 0.0;
    double max_weight = 0.0;
    for (int t=0; t<num_time_steps_; ++t)
    {
      weight = time_step_weights_[d][t];
      weight_sum += weight;
      parameter_updates_[d](0,t) *= weight;
      if (weight > max_weight)
        max_weight = weight;
    }
    if (weight_sum < 1e-6)
      weight_sum = 1e-6;

    double divisor = weight_sum/num_time_steps_;

    if (max_weight > divisor)
    {
      divisor = max_weight;
    }
    parameter_updates_[d].row(0) /= divisor;

    parameter_updates_[d].row(0).transpose() = projection_matrix_[d]*parameter_updates_[d].row(0).transpose();

  }

  return true;
}

bool PolicyImprovement::improvePolicy(std::vector<Eigen::MatrixXd>& parameter_updates)
{
    ROS_ASSERT(initialized_);

    //ros::WallTime start_time = ros::WallTime::now();
    //computeRolloutCumulativeCosts();
    //ROS_INFO("Cumulative costs took %f seconds", (ros::WallTime::now() - start_time).toSec());
    //start_time = ros::WallTime::now();
    computeRolloutProbabilities();
    //ROS_INFO("Probabilities took %f seconds", (ros::WallTime::now() - start_time).toSec());
    //start_time = ros::WallTime::now();
    computeParameterUpdates();
    //ROS_INFO("Updates took %f seconds", (ros::WallTime::now() - start_time).toSec());
    parameter_updates = parameter_updates_;

    return true;
}

bool PolicyImprovement::preAllocateTempVariables()
{
    tmp_noise_.clear();
    tmp_parameters_.clear();
    parameter_updates_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
        tmp_noise_.push_back(VectorXd::Zero(num_parameters_[d]));
        tmp_parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
        parameter_updates_.push_back(MatrixXd::Zero(num_time_steps_, num_parameters_[d]));
        time_step_weights_.push_back(VectorXd::Zero(num_time_steps_));
    }
    tmp_max_cost_ = VectorXd::Zero(num_time_steps_);
    tmp_min_cost_ = VectorXd::Zero(num_time_steps_);
    tmp_sum_rollout_probabilities_ = VectorXd::Zero(num_time_steps_);

    return true;
}

bool PolicyImprovement::preComputeProjectionMatrices()
{
  projection_matrix_.resize(num_dimensions_);
  inv_projection_matrix_.resize(num_dimensions_);

  if (!use_projection_)
  {
    // just use identities
    for (int d=0; d<num_dimensions_; ++d)
    {
      projection_matrix_[d] = Eigen::MatrixXd::Identity(num_parameters_[d], num_parameters_[d]);
      inv_projection_matrix_[d] = projection_matrix_[d];
    }

    return true;
  }

//  ROS_INFO("Precomputing projection matrices..");
  for (int d=0; d<num_dimensions_; ++d)
  {
    projection_matrix_[d] = inv_control_costs_[d];

    // scale each column separately - divide by max element
//    for (int p=0; p<num_parameters_[d]; ++p)
//    {
//      double column_max = fabs(inv_control_costs_[d](0,p));
//      for (int p2 = 1; p2 < num_parameters_[d]; ++p2)
//      {
//        if (fabs(inv_control_costs_[d](p2,p)) > column_max)
//          column_max = fabs(inv_control_costs_[d](p2,p));
//      }
//      projection_matrix_[d].col(p) *= (1.0/(num_parameters_[d]*column_max));
//    }

    // scale each column separately - divide by diagonal element
    for (int p=0; p<num_parameters_[d]; ++p)
    {
      double column_max = projection_matrix_[d](p,p);
      projection_matrix_[d].col(p) *= (1.0/(num_parameters_[d]*column_max));
      //projection_matrix_[d].col(p) *= (1.0/(num_parameters_[d]*column_max));
      //printf("column %d max = %f\n", p, column_max);
    }

//    double max_entry = inv_control_costs_[d].maxCoeff();
//    projection_matrix_[d] /= max_entry*num_parameters_[d];

    //ROS_INFO_STREAM("Projection matrix = \n" << projection_matrix_[d]);
    inv_projection_matrix_[d] = projection_matrix_[d].fullPivLu().inverse();
  }
//  ROS_INFO("Done precomputing projection matrices.");
  return true;
}

bool PolicyImprovement::computeNoise(Rollout& rollout)
{
    for (int d=0; d<num_dimensions_; ++d)
    {
        rollout.noise_[d] =  rollout.parameters_noise_[d] - rollout.parameters_[d];
    }
    return true;
}

bool PolicyImprovement::computeRolloutControlCosts(Rollout& rollout)
{
    policy_->computeControlCosts(rollout.parameters_, rollout.noise_projected_,
                                 control_cost_weight_, rollout.control_costs_);
    return true;
}

bool PolicyImprovement::copyParametersFromPolicy()
{
    if (!policy_->getParameters(parameters_))
    {
        ROS_ERROR("Failed to get policy parameters.");
        return false;
    }
    return true;
}

bool PolicyImprovement::getTimeStepWeights(std::vector<Eigen::VectorXd>& time_step_weights)
{
  time_step_weights = time_step_weights_;
  return true;
}

void PolicyImprovement::getAllRollouts(std::vector<Rollout>& rollouts)
{
  rollouts.resize(num_rollouts_);
  for (int i=0; i<num_rollouts_; ++i)
  {
    rollouts[i] = rollouts_[i];
  }
}

void PolicyImprovement::getNoiselessRollout(Rollout& rollout)
{
  rollout = noiseless_rollout_;
}

void PolicyImprovement::getAdaptedStddevs(std::vector<double>& stddevs)
{
  stddevs = adapted_stddevs_;
}

void PolicyImprovement::setCostCumulation(bool use_cumulative_costs)
{
  use_cumulative_costs_ = use_cumulative_costs;
}

void PolicyImprovement::resetAdaptiveNoise()
{
  adapted_covariance_valid_ = false;
}

};
