/**
 * @file polynomial.h
 * @brief TODO
 *
 * @author Jorge Nicho
 * @date Jan 6, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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

#ifndef INCLUDE_STOMP_MOVEIT_UTILS_POLYNOMIAL_H_
#define INCLUDE_STOMP_MOVEIT_UTILS_POLYNOMIAL_H_

#include <stomp_core/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <Eigen/Core>

/**
 * @namespace stomp_moveit
 */
namespace stomp_moveit
{

/**
 * @namespace utils
 */
namespace utils
{

/**
 * @namespace smoothing
 */
namespace polynomial
{

  /**
   * @brief Fit a polynomial with fixed indices
   * @param d Degree of the polynomial
   * @param x The domain values
   * @param y Fit values
   * @param fixed_indices Indicies of the input data to remain fixed during fit.
   * @param poly_params Polynomial parameters found during the fit
   * @return The new fitted values.
   */
  Eigen::VectorXd polyFitWithFixedPoints(const int d,
                                         const Eigen::VectorXd &x,
                                         const Eigen::VectorXd &y,
                                         const Eigen::VectorXi &fixed_indices,
                                         Eigen::VectorXd &poly_params);

  /**
   * @brief Get the polynomial smoother domain values.
   *
   * The domain values is time. The time is calculated finding the joint
   * that takes the longest given the total distance traveled and its max
   * velocity. This is the dominating joint therefore its time is used as
   * the domain values for the smoothing operation.
   *
   * @param joint_models  List of active joint models
   * @param parameters    The parameters generated in the previous iteration [num_dimensions x num_timesteps]
   * @param domain_values The time domain values for the trajectory [num_timesteps]
   */
  void generateMinimumDomainValues(const std::vector<const  moveit::core::JointModel*> joint_models, const Eigen::MatrixXd &parameters,
                                   Eigen::VectorXd &domain_values);

  /**
   * @brief Generates a Vandermonde matrix from the domain values as described here https://en.wikipedia.org/wiki/Vandermonde_matrix. It
   * uses the <b>poly_order</b> parameter in order to set the size of the matrix.
   * @param domain_vals The domain values
   * @param order       The order of the polynomial
   * @param v           The output matrix
   */
  void fillVandermondeMatrix(const Eigen::ArrayXd &domain_vals, const int &order, Eigen::MatrixXd &v);

  /**
   * @brief Applies a polynomial smoothing method to a trajectory.  It checks for joint limits and makes corrections when these are exceeded as a
   *        result of the smoothing process.
   * @param robot_model
   * @param group_name
   * @param parameters
   * @param poly_order
   * @param smoothing_attempts
   * @param joint_limit_margin
   * @return
   */
  bool applyPolynomialSmoothing(moveit::core::RobotModelConstPtr robot_model, const std::string& group_name, Eigen::MatrixXd& parameters,
                                       int poly_order = 5, int smoothing_attempts = 5, double joint_limit_margin = 1e-5);

} // end of namespace smoothing
} // end of namespace utils
} // end of namespace stomp_moveit


#endif /* INCLUDE_STOMP_MOVEIT_UTILS_POLYNOMIAL_H_ */
