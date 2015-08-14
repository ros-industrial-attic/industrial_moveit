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

#include <stomp/covariant_movement_primitive.h>
#include <stomp/stomp_utils.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <sstream>

using namespace Eigen;

namespace stomp
{

CovariantMovementPrimitive::CovariantMovementPrimitive()
{
}

CovariantMovementPrimitive::~CovariantMovementPrimitive()
{
}

bool CovariantMovementPrimitive::initialize(const int num_time_steps,
                                            const int num_dimensions,
                                            const double movement_duration,
                                            const std::vector<Eigen::MatrixXd>& derivative_costs,
                                            const std::vector<Eigen::VectorXd>& initial_trajectory)
{
  num_time_steps_ = num_time_steps;
  num_dimensions_ = num_dimensions;
  movement_duration_ = movement_duration;
  derivative_costs_ = derivative_costs;
  parameters_all_ = initial_trajectory;

  STOMP_VERIFY(initializeVariables());
  STOMP_VERIFY(initializeCosts());
  STOMP_VERIFY(initializeBasisFunctions());

  return true;
}

bool CovariantMovementPrimitive::setToMinControlCost()
{
  computeMinControlCostParameters();
  return true;
}

bool CovariantMovementPrimitive::computeLinearControlCosts()
{
  linear_control_costs_.clear();
  linear_control_costs_.resize(num_dimensions_, VectorXd::Zero(num_vars_free_));
  constant_control_costs_.clear();
  constant_control_costs_.resize(num_dimensions_, 0.0);

  for (int d=0; d<num_dimensions_; ++d)
  {
    linear_control_costs_[d].transpose() = parameters_all_[d].segment(0, DIFF_RULE_LENGTH-1).transpose() *
        control_costs_all_[d].block(0, free_vars_start_index_, DIFF_RULE_LENGTH-1, num_vars_free_);
    linear_control_costs_[d].transpose() += parameters_all_[d].segment(free_vars_end_index_+1, DIFF_RULE_LENGTH-1).transpose() *
        control_costs_all_[d].block(free_vars_end_index_+1, free_vars_start_index_, DIFF_RULE_LENGTH-1, num_vars_free_);
    linear_control_costs_[d] *= 2.0; // because the cost matrix is symmetric

    // the next term is from the (x - x_desired)^2 part:
    linear_control_costs_[d] += - movement_dt_ * 2.0*(parameters_all_[d].segment(free_vars_start_index_, num_vars_free_).array()*
        derivative_costs_[d].block(free_vars_start_index_, 0, num_vars_free_, 1).array()).matrix();


    // get the constant parts
    Eigen::VectorXd const_params = Eigen::VectorXd::Zero(2*TRAJECTORY_PADDING);
    Eigen::MatrixXd const_matrix = Eigen::MatrixXd::Zero(2*TRAJECTORY_PADDING, 2*TRAJECTORY_PADDING);
    const_params.head(TRAJECTORY_PADDING) = parameters_all_[d].segment(0, TRAJECTORY_PADDING);
    const_params.tail(TRAJECTORY_PADDING) = parameters_all_[d].segment(free_vars_end_index_+1, TRAJECTORY_PADDING);
    const_matrix.topLeftCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING) = control_costs_all_[d].topLeftCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING);
    const_matrix.bottomRightCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING) = control_costs_all_[d].bottomRightCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING);
    const_matrix.topRightCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING) = control_costs_all_[d].topRightCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING);
    const_matrix.bottomLeftCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING) = control_costs_all_[d].bottomLeftCorner(TRAJECTORY_PADDING, TRAJECTORY_PADDING);

    constant_control_costs_[d] = movement_dt_ * const_params.transpose() * const_matrix * const_params;

  }


  return true;
}

bool CovariantMovementPrimitive::computeMinControlCostParameters()
{
  for (int d=0; d<num_dimensions_; ++d)
  {
    parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) =
        -0.5 * inv_control_costs_[d] * linear_control_costs_[d];
  }
  min_control_cost_parameters_all_ = parameters_all_;
  min_control_cost_parameters_free_.resize(num_dimensions_);
  for (int d=0; d<num_dimensions_; ++d)
  {
    min_control_cost_parameters_free_[d] = min_control_cost_parameters_all_[d].segment(free_vars_start_index_, num_vars_free_);
  }
  return true;
}

bool CovariantMovementPrimitive::initializeVariables()
{
  movement_dt_ = movement_duration_ / (num_time_steps_ + 1);

  num_vars_free_ = num_time_steps_;
  num_vars_all_ = num_vars_free_ + 2*(DIFF_RULE_LENGTH-1);
  free_vars_start_index_ = DIFF_RULE_LENGTH - 1;
  free_vars_end_index_ = free_vars_start_index_ + num_vars_free_ - 1;

  num_parameters_.clear();
  for (int d=0; d<num_dimensions_; ++d)
    num_parameters_.push_back(num_time_steps_);

  ROS_ASSERT(parameters_all_.size() == num_dimensions_);
  ROS_ASSERT(derivative_costs_.size() == num_dimensions_);
  for (int d=0; d<num_dimensions_; ++d)
  {
    ROS_ASSERT(parameters_all_[d].size() == num_vars_all_);
    ROS_ASSERT(derivative_costs_[d].cols() == NUM_DIFF_RULES);
    ROS_ASSERT(derivative_costs_[d].rows() == num_vars_all_);
  }

  return true;
}

bool CovariantMovementPrimitive::initializeCosts()
{
  createDifferentiationMatrices();

  control_costs_all_.clear();
  control_costs_.clear();
  inv_control_costs_.clear();
  derivative_costs_sqrt_.clear();
  for (int d=0; d<num_dimensions_; ++d)
  {
    // get sqrt of derivative costs
    derivative_costs_sqrt_.push_back(derivative_costs_[d].array().sqrt().matrix());

    // construct the quadratic cost matrices (for all variables)
    MatrixXd cost_all = MatrixXd::Zero(num_vars_all_, num_vars_all_);
    //MatrixXd cost_all = MatrixXd::Identity(num_vars_all_, num_vars_all_) * cost_ridge_factor_;
    for (int i=0; i<NUM_DIFF_RULES; ++i)
    {
      cost_all += movement_dt_ * (differentiation_matrices_[i].transpose() *
          derivative_costs_[d].col(i).asDiagonal() *
          differentiation_matrices_[i]);
    }
    control_costs_all_.push_back(cost_all);

    // extract the quadratic cost just for the free variables:
    MatrixXd cost_free = cost_all.block(DIFF_RULE_LENGTH-1, DIFF_RULE_LENGTH-1, num_vars_free_, num_vars_free_);
    control_costs_.push_back(cost_free);

    inv_control_costs_.push_back(cost_free.fullPivLu().inverse());
  }

  computeLinearControlCosts();

  return true;
}

bool CovariantMovementPrimitive::initializeBasisFunctions()
{
  basis_functions_.clear();
  for (int d=0; d<num_dimensions_; ++d)
  {
    basis_functions_.push_back(MatrixXd::Identity(num_vars_free_, num_vars_free_));
  }
  return true;
}

void CovariantMovementPrimitive::createDifferentiationMatrices()
{
  differentiation_matrices_.clear();
  differentiation_matrices_.resize(NUM_DIFF_RULES, MatrixXd::Zero(num_vars_all_, num_vars_all_));

  for (int d=0; d<NUM_DIFF_RULES; ++d)
  {
    stomp::getDifferentiationMatrix(num_vars_all_, stomp::CostComponents(d), movement_dt_, differentiation_matrices_[d]);
  }
}

bool CovariantMovementPrimitive::getDerivatives(int derivative_number, std::vector<Eigen::VectorXd>& derivatives) const
{
  derivatives.resize(num_dimensions_);
  for (int dim=0; dim<num_dimensions_; ++dim)
  {
    derivatives[dim] = (differentiation_matrices_[derivative_number] * parameters_all_[dim]).segment(free_vars_start_index_, num_vars_free_);
  }
  return true;
}

bool CovariantMovementPrimitive::computeControlCostGradient(const std::vector<Eigen::VectorXd>& parameters,
                                const double weight,
                                std::vector<Eigen::VectorXd>& gradient)
{
  gradient.resize(num_dimensions_, Eigen::VectorXd::Zero(num_vars_free_));

  for (int d=0; d<num_dimensions_; ++d)
  {
    gradient[d] = weight * (2.0 * control_costs_[d] * parameters[d] + linear_control_costs_[d]);
  }

  return true;
}

bool CovariantMovementPrimitive::computeControlCosts(const std::vector<Eigen::VectorXd>& parameters,
                                                     const std::vector<Eigen::VectorXd>& noise,
                                                     const double weight,
                                                     std::vector<Eigen::VectorXd>& control_costs)
{
  //printf("Weight = %f\n", weight);
  // this uses the already squared control cost matrix
  for (int d=0; d<num_dimensions_; ++d)
  {
    VectorXd params_all = parameters_all_[d];
    VectorXd params_free = parameters[d] + noise[d];
    VectorXd costs_all = VectorXd::Zero(num_vars_all_);
    params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d] + noise[d];

//    costs_all = weight * (params_all.array() *
//        (control_costs_all_[d] * params_all).array()).matrix();
//    control_costs[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
//
//    for (int i=0; i<free_vars_start_index_; ++i)
//    {
//      control_costs[d](0) += costs_all(i);
//      control_costs[d](num_vars_free_-1) += costs_all(num_vars_all_-(i+1));
//    }

    // compute costs using control_costs_all
//    double costs_all = weight * params_all.transpose() * control_costs_all_[d] * params_all;
//    control_costs[d] = Eigen::VectorXd::Ones(num_vars_free_) * costs_all;

    // compute costs using control_costs_free and linear_costs (constant per time-step)
//    double costs = params_free.transpose() * control_costs_[d] * params_free;
//    costs += linear_control_costs_[d].transpose() * params_free;
//    costs += constant_control_costs_[d];
//    control_costs[d] = (1.0/num_parameters_[d]) * weight * costs * Eigen::VectorXd::Ones(num_vars_free_);


    // compute them from the original diff matrices, per timestep
    for (int i=0; i<NUM_DIFF_RULES; ++i)
    {
      Eigen::ArrayXXd Ax = (differentiation_matrices_[i] * params_all).array() *
          derivative_costs_sqrt_[d].col(i).array();
      costs_all += movement_dt_ * weight * (Ax * Ax).matrix();
    }
    control_costs[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
    //control_costs[d] = Eigen::VectorXd::Zero(num_vars_free_); // TODO FIXME we don't use the cost above for now
    for (int i=0; i<free_vars_start_index_; ++i)
    {
      control_costs[d](0) += costs_all(i);
      control_costs[d](num_vars_free_-1) += costs_all(num_vars_all_-(i+1));
    }


    //printf("Control costs for dim %d = %f\n", d, control_costs[d].sum());

    // add linear costs:
//    control_costs[d] += weight * (linear_control_costs_[d].array() *
//        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_).array()).matrix();

  }


  // this measures the accelerations and squares them
//  for (int d=0; d<num_dimensions_; ++d)
//  {
//    VectorXd params_all = parameters_all_[d];
//    VectorXd costs_all = VectorXd::Zero(num_vars_all_);
//
//    params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d] + noise[d];
//    VectorXd acc_all = VectorXd::Zero(num_vars_all_);
//    for (int i=0; i<NUM_DIFF_RULES; ++i)
//    {
//      acc_all = differentiation_matrices_[i]*params_all;
//      costs_all += weight * derivative_costs_[i] * (acc_all.array()*acc_all.array()).matrix();
//    }
//
//    control_costs[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
//    for (int i=0; i<free_vars_start_index_; ++i)
//    {
//      control_costs[d](0) += costs_all(i);
//      control_costs[d](num_vars_free_-1) += costs_all(num_vars_all_-(i+1));
//    }
//  }


  return true;
}


//bool CovariantMovementPrimitive::computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<std::vector<Eigen::VectorXd> >& parameters,
//                                                     const double weight, std::vector<Eigen::VectorXd>& control_costs)
//{
//  //Policy::computeControlCosts(control_cost_matrices, parameters, weight, control_costs);
//
//  // we use the locally stored control costs
//
//  // this uses the already squared control cost matrix
//  /*for (int d=0; d<num_dimensions_; ++d)
//    {
//        control_costs[d] = VectorXd::Zero(num_time_steps_);
//        VectorXd params_all = parameters_all_[d];
//        for (int t=0; t<num_time_steps_; ++t)
//        {
//            params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d][t];
//            VectorXd r_times_u = control_costs_all_[d] * params_all;
//            control_costs[d] += weight * (r_times_u.segment(free_vars_start_index_, num_vars_free_).cwise() * parameters[d][t]);
//        }
//    }*/
//
//
//  // this measures the accelerations and squares them
//  for (int d=0; d<num_dimensions_; ++d)
//  {
//    VectorXd params_all = parameters_all_[d];
//    VectorXd costs_all = VectorXd::Zero(num_vars_all_);
//    for (int t=0; t<num_time_steps_; ++t)
//    {
//      params_all.segment(free_vars_start_index_, num_vars_free_) = parameters[d][t];
//      VectorXd acc_all = VectorXd::Zero(num_vars_all_);
//      for (int i=0; i<NUM_DIFF_RULES; ++i)
//      {
//        acc_all = differentiation_matrices_[i]*params_all;
//        costs_all += weight * derivative_costs_[i] * (acc_all.array()*acc_all.array()).matrix();
//      }
//    }
//    control_costs[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
//    for (int i=0; i<free_vars_start_index_; ++i)
//    {
//      control_costs[d](0) += costs_all(i);
//      control_costs[d](num_vars_free_-1) += costs_all(num_vars_all_-(i+1));
//    }
//  }
//
//
//  return true;
//}

bool CovariantMovementPrimitive::updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights)
{
  ROS_ASSERT(int(updates.size()) == num_dimensions_);

  // this takes only the diagonal elements
  /*for (int d=0; d<num_dimensions_; ++d)
    {
        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += updates[d].diagonal();
    }*/

  // this averages all the updates
  //double divisor = 1.0 / num_vars_free_;
  double divisor = 1.0;
  for (int d=0; d<num_dimensions_; ++d)
  {
    parameters_all_[d].segment(free_vars_start_index_, num_vars_free_).transpose() +=
        divisor * updates[d].row(0);
  }

  //    for (int d=0; d<num_dimensions_; ++d)
  //    {
  //      double weight = 0.0;
  //      double weight_sum = 0.0;
  //
  //      Eigen::VectorXd update = Eigen::VectorXd::Zero(num_vars_free_);
  //      for (int t=0; t<num_time_steps_; ++t)
  //      {
  //          weight = time_step_weights[d][t];
  //          weight_sum += weight;
  //          update.transpose() += updates[d].row(t) * weight;
  //          //ROS_INFO_STREAM("Update at time " << t << " = " << updates[d].row(t));
  //      }
  //      if (weight_sum <1e-6)
  //        weight_sum = 1e-6;
  //      parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += (1.0/weight_sum)*update;
  //    }

  // this weights updates by number of time-steps remaining:
  //    for (int d=0; d<num_dimensions_; ++d)
  //    {
  //        double weight=0.0;
  //        double weight_sum=0.0;
  //        Eigen::VectorXd update = Eigen::VectorXd::Zero(num_vars_free_);
  //        for (int t=0; t<num_time_steps_; ++t)
  //        {
  //            weight = double(num_time_steps_ - t);
  //            weight_sum += weight;
  //            update.transpose() += updates[d].row(t) * weight;
  //            //ROS_INFO_STREAM("Update at time " << t << " = " << updates[d].row(t));
  //        }
  //        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) += (1.0/weight_sum)*update;
  //    }

  return true;
}

//bool CovariantMovementPrimitive::readFromDisc(const std::string abs_file_name)
//{
//    // TODO: implement this
//    return true;
//}

bool CovariantMovementPrimitive::writeToFile(const std::string abs_file_name)
{
  FILE *f;
  f = fopen(abs_file_name.c_str(), "w");
  if (!f)
    return false;

  for (int i=free_vars_start_index_-1; i<=free_vars_end_index_+1; ++i)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      fprintf(f,"%f\t", parameters_all_[d](i));
    }
    fprintf(f,"\n");
  }

  fclose(f);
  return true;
}

double CovariantMovementPrimitive::getMovementDuration() const
{
  return movement_duration_;
}

double CovariantMovementPrimitive::getMovementDt() const
{
  return movement_dt_;
}

const Eigen::MatrixXd& CovariantMovementPrimitive::getDifferentiationMatrix(int derivative_number) const
{
  return differentiation_matrices_[derivative_number];
}

}
