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
 * stomp_utils.h
 *
 *  Created on: Mar 7, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_

#include <string>
#include <vector>
#include <Eigen/Core>

namespace stomp_core
{

struct NoiseGeneration
{
  enum NoiseUpdateMethod
  {
    CONSTANT = 0, ADAPTIVE, EXPONENTIAL_DECAY
  };

  std::vector<double> stddev;
  std::vector<double> decay;
  std::vector<double> min_stddev;
  int method; /**< method used to update the standard deviation values.  */
  double update_rate; /**< used when using  KL divergence adaptation, should stay within values of (0,1] */
};

struct Rollout
{
  Eigen::MatrixXd noise;                       /**< [num_dimensions] x num_time_steps, random noise applied to the parameters*/
  Eigen::MatrixXd parameters_noise;            /**< [num_dimensions] x num_time_steps, the sum of parameters + noise */

  Eigen::VectorXd state_costs; /**< num_time_steps */
  Eigen::MatrixXd control_costs; /**< [num_dimensions] x num_time_steps */
  Eigen::MatrixXd total_costs; /**< [num_dimensions] x num_time_steps total_cost[d] = state_costs_ + control_costs_[d]*/
  Eigen::MatrixXd cumulative_costs; /**< [num_dimensions] x num_time_steps cumulative_costs[d] = Ones(num_time_steps) * total_costs_[d].sum();*/
  Eigen::MatrixXd probabilities; /**< [num_dimensions] x num_time_steps */

  std::vector<double> full_probabilities; /**< [num_dimensions] probabilities of full trajectory */
  std::vector<double> full_costs; /**< [num_dimensions] state_cost + control_cost for each joint over the entire trajectory
                                       full_costs_[d] = state_cost.sum() + control_cost[d].sum() */

  double importance_weight; /**< importance sampling weight */
  double total_cost; /**< combined state + control cost over the entire trajectory for all joints */

};

namespace DerivativeOrders
{
  enum DerivativeOrder
  {
    STOMP_POSITION = 0, STOMP_VELOCITY = 1, STOMP_ACCELERATION = 2, STOMP_JERK = 3
  };
}
;

static const int FINITE_DIFF_RULE_LENGTH = 7;
static const double FINITE_DIFF_COEFFS[FINITE_DIFF_RULE_LENGTH][FINITE_DIFF_RULE_LENGTH] = { {0, 0, 0, 1, 0, 0, 0}, // position
    {0, 0, -1, 1, 0, 0, 0}, // velocity (backward difference)
    {0, -1 / 12.0, 16 / 12.0, -30 / 12.0, 16 / 12.0, -1 / 12.0, 0}, // acceleration (five point stencil)
    {0, 1 / 12.0, -17 / 12.0, 46 / 12.0, -46 / 12.0, 17 / 12.0, -1 / 12.0} // jerk
};

bool generateFiniteDifferenceMatrix(int num_time_steps, DerivativeOrders::DerivativeOrder order, double dt,
                                    Eigen::MatrixXd& diff_matrix);

void differentiate(const Eigen::VectorXd& parameters, DerivativeOrders::DerivativeOrder order,
                          double dt, Eigen::VectorXd& derivatives );

void toVector(const Eigen::MatrixXd& m,std::vector<Eigen::VectorXd>& v);
std::string toString(const std::vector<Eigen::VectorXd>& data);
std::string toString(const Eigen::VectorXd& data);
std::string toString(const Eigen::MatrixXd& data);



} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_ */
