/**
 * @file utils.h
 * @brief This is a utility class for stomp
 *
 * @author Jorge Nicho
 * @date March 7, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_

#include <string>
#include <vector>
#include <Eigen/Core>

namespace stomp_core
{

/** @brief The data structure used to store information about a single rollout. */
struct Rollout
{
  Eigen::MatrixXd noise;                   /**< @brief A matrix [num_dimensions][num_time_steps] of random noise applied to the parameters*/
  Eigen::MatrixXd parameters_noise;        /**< @brief A matrix [num_dimensions][num_time_steps] of the sum of parameters + noise */

  Eigen::VectorXd state_costs;             /**< @brief A vector [num_time_steps] of the cost at each timestep */
  Eigen::MatrixXd control_costs;           /**< @brief A matrix [num_dimensions][num_time_steps] of the control cost for each parameter at every timestep */
  Eigen::MatrixXd total_costs;             /**< @brief A matrix [num_dimensions][num_time_steps] of the total cost, where total_cost[d] = state_costs_ + control_costs_[d]*/
  Eigen::MatrixXd probabilities;           /**< @brief A matrix [num_dimensions][num_time_steps] of the probability for each parameter at every timestep */

  std::vector<double> full_probabilities; /**< @brief A vector [num_dimensions] of the probabilities for the full trajectory */
  std::vector<double> full_costs;         /**< @brief A vector [num_dimensions] of the full coss, state_cost + control_cost for each joint over the entire trajectory
                                               full_costs_[d] = state_cost.sum() + control_cost[d].sum() */

  double importance_weight;               /**< @brief importance sampling weight */
  double total_cost;                      /**< @brief combined state + control cost over the entire trajectory for all joints */

};


namespace DerivativeOrders
{
/** @brief Available finite differentiation methods */
enum DerivativeOrder
{
  STOMP_POSITION = 0,     /**< Calculate position using finite differentiation */
  STOMP_VELOCITY = 1,     /**< Calculate velocity using finite differentiation */
  STOMP_ACCELERATION = 2, /**< Calculate acceleration using finite differentiation */
  STOMP_JERK = 3          /**< Calculate jerk using finite differentiation */
};
}

namespace TrajectoryInitializations
{
/** @brief Available trajectory initialization methods */
enum TrajectoryInitialization
{
  LINEAR_INTERPOLATION = 1,       /**< Calculate initial trajectory using linear interpolation */
  CUBIC_POLYNOMIAL_INTERPOLATION, /**< Calculate initial trajectory using cubic polynomial interpolation */
  MININUM_CONTROL_COST            /**< Calculate initial trajectory using minimum control cost */
};
}

/** @brief The data structure used to store STOMP configuration parameters. */
struct StompConfiguration
{
  // General settings
  int num_iterations;                    /**< @brief Maximum number of iteration allowed */
  int num_iterations_after_valid;        /**< @brief Stomp will stop optimizing this many iterations after finding a valid solution */
  int num_timesteps;                     /**< @brief Number of timesteps */
  int num_dimensions;                    /**< @brief Parameter dimensionality */
  double delta_t;                        /**< @brief Time change between consecutive points */
  int initialization_method;             /**< @brief TrajectoryInitializations::TrajectoryInitialization */

  // Probability Calculation
  double exponentiated_cost_sensitivity; /**< @brief Default exponetiated cost sensitivity coefficient */

  // Noisy trajectory generation
  int num_rollouts;                      /**< @brief Number of noisy trajectories*/
  int max_rollouts;                      /**< @brief The combined number of new and old rollouts during each iteration shouldn't exceed this value */

  // Cost calculation
  double control_cost_weight;            /**< @brief Percentage of the trajectory accelerations cost to be applied in the total cost calculation >*/
};

/** @brief The number of columns in the finite differentiation rule */
static const int FINITE_DIFF_RULE_LENGTH = 7;

/** @brief Contains the coefficients for each of the finite central differentiation (position, velocity, acceleration, and jerk) */
static const double FINITE_CENTRAL_DIFF_COEFFS[FINITE_DIFF_RULE_LENGTH][FINITE_DIFF_RULE_LENGTH] = {
    {0, 0        , 0        , 1        , 0        , 0         , 0      }, // position
    {0, 1.0/12.0 , -2.0/3.0 , 0        , 2.0/3.0  , -1.0/12.0 , 0      }, // velocity
    {0, -1/12.0  , 16/12.0  , -30/12.0 , 16/12.0  , -1/12.0   , 0      }, // acceleration (five point stencil)
    {0, 1/12.0   , -17/12.0 , 46/12.0  , -46/12.0 , 17/12.0   , -1/12.0}  // jerk
};

/** @brief Contains the coefficients for each of the finite forward differentiation (position, velocity, acceleration, and jerk) */
static const double FINITE_FORWARD_DIFF_COEFFS[FINITE_DIFF_RULE_LENGTH][FINITE_DIFF_RULE_LENGTH] = {
    {1          , 0         , 0         , 0      , 0        , 0         , 0    }, // position
    {-25.0/12.0 , 4.0       , -3.0      , 4.0/3.0, -1.0/4.0 , 0         , 0    }, // velocity
    {15.0/4.0   , -77.0/6.0 , 107.0/6.0 , -13.0  , 61.0/12.0, -5.0/6.0  , 0    }, // acceleration (five point stencil)
    {-49/8      , 29        , -461/8    , 62     , -307/8   , 13        , -15/8}  // jerk
};

/**
 * @brief Generate a finite difference matrix based on the input DerivativeOrder
 * @param num_time_steps The number of timesteps
 * @param order          The differentiation order
 * @param dt             The timestep in seconds
 * @param diff_matrix    The generated finite difference matrix
 */
void generateFiniteDifferenceMatrix(int num_time_steps, DerivativeOrders::DerivativeOrder order, double dt,
                                    Eigen::MatrixXd& diff_matrix);

/**
 * @brief Differentiates the input parameters based on the DerivativeOrder.
 * @param parameters  The parameters to be differentiated
 * @param order       The differentiation order
 * @param dt          The timestep in seconds
 * @param derivatives The differentiation of the input parameters
 */
void differentiate(const Eigen::VectorXd& parameters, DerivativeOrders::DerivativeOrder order,
                          double dt, Eigen::VectorXd& derivatives );

/**
 * @brief Generate a smoothing matrix M
 * @param num_time_steps       The number of timesteps
 * @param dt                   The timestep in seconds
 * @param projection_matrix_M  The smoothing matrix
 */
void generateSmoothingMatrix(int num_time_steps, double dt, Eigen::MatrixXd& projection_matrix_M);

/**
 * @brief Convert a Eigen::MatrixXd to a std::vector<Eigen::VectorXd>
 * Each element in the std::vector represents a row in the Eigen::MatrixXd
 *
 * @param m The matrix to be converted
 * @param v The returned std::vector<Eigen::VectorXd>
 */
void toVector(const Eigen::MatrixXd& m,std::vector<Eigen::VectorXd>& v);

/**
 * @brief Convert std::vector<Eigen::VectorXd> formated string
 * @param data The item to be converted
 * @return Formated string representing the input object
 */
std::string toString(const std::vector<Eigen::VectorXd>& data);

/**
 * @brief Convert an Eigen::VectorXd to a formated string
 * @param data The item to be converted
 * @return Formated string representing data
 */
std::string toString(const Eigen::VectorXd& data);

/**
 * @brief Convert an Eigen::MatrixXd to a formated string
 * @param data The item to be converted
 * @return Formated string representing data
 */
std::string toString(const Eigen::MatrixXd& data);

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_ */
