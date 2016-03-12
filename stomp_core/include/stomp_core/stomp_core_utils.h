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
    CONSTANT = 0,
    ADAPTIVE ,
    EXPONENTIAL_DECAY
  };

  std::vector<double> stddev;
  std::vector<double> decay;
  std::vector<double> min_stddev;
  int method;                           /**< method used to update the standard deviation values.  */
  double update_rate;                   /**< used when using  KL divergence adaptation, should stay within values
                                             of (0,1] */

  std::vector<double> updated_stddev;   /**< To be used for storing the updated stddev values during each iteration */
};

struct Rollout
{
    std::vector<Eigen::VectorXd> noise;                 /**< [num_dimensions] num_time_steps, random noise applied to the parameters*/
    std::vector<Eigen::VectorXd> parameters_noise;     /**< [num_dimensions] num_time_steps, the sum of parameters + noise */

    Eigen::VectorXd state_costs;                        /**< num_time_steps */
    std::vector<Eigen::VectorXd> control_costs;         /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> total_costs;           /**< [num_dimensions] num_time_steps
                                                             total_cost[d] = state_costs_ + control_costs_[d]*/
    std::vector<Eigen::VectorXd> cumulative_costs;      /**< [num_dimensions] num_time_steps
                                                              cumulative_costs[d] = Ones(num_time_steps) * total_costs_[d].sum();*/
    std::vector<Eigen::VectorXd> probabilities;         /**< [num_dimensions] num_time_steps */

    std::vector<double> full_probabilities;             /**< [num_dimensions] probabilities of full trajectory */
    std::vector<double> full_costs;                     /**< [num_dimensions] state_cost + control_cost for each joint
                                                             full_costs_[d] = state_cost.sum() + control_cost[d].sum() */

    double importance_weight;                           /**< importance sampling weight */
    double total_cost;                                  /**< state + control cost over the entire trajectory */

};

namespace DerivativeOrders
{
  enum DerivativeOrder
  {
    STOMP_POSITION = 0,
    STOMP_VELOCITY = 1,
    STOMP_ACCELERATION = 2,
    STOMP_JERK = 3
  };
};

static const int FINITE_DIFF_RULE_LENGTH = 7;
static const double FINITE_DIFF_COEFFS[FINITE_DIFF_RULE_LENGTH][FINITE_DIFF_RULE_LENGTH] =
{
    {0,       0,        0,        1,        0,       0,       0},       // position
    {0,       0,       -1,        1,        0,       0,       0},       // velocity (backward difference)
    {0, -1/12.0,  16/12.0, -30/12.0,  16/12.0, -1/12.0,       0},       // acceleration (five point stencil)
    {0,  1/12.0, -17/12.0,  46/12.0, -46/12.0, 17/12.0, -1/12.0}        // jerk
};

bool generateFiniteDifferenceMatrix(int num_time_steps,
                                           DerivativeOrders::DerivativeOrder order,
                                           double dt, Eigen::MatrixXd& diff_matrix);





} /* namespace stomp */


#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_ */
