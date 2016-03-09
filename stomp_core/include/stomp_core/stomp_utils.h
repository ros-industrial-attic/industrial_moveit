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

namespace stomp_core
{

namespace NoiseGenerationMethods
{
  enum NoiseGenerationMethod
  {
    NONE = 0,
    KL_DIVERGENCE_ADAPTATION = 1
  };
}

struct NoiseGenerationParams
{
  std::vector<double> stddev;
  std::vector<double> decay;
  std::vector<double> min_stddev;
  int method;                           /**< method used to update the standard deviation values.  */
  double update_rate;                   /**< used when using  KL divergence adaptation should stay within values
                                             of (0,1] */

  std::vector<double> updated_stddev;   /**< To be used for storing the updated stddev values during each iteration */
};

struct Rollout
{
    std::vector<Eigen::VectorXd> parameters;            /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> noise;                 /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> parameters_noise_;     /**< [num_dimensions] num_time_steps */
    Eigen::VectorXd state_costs;                        /**< num_time_steps */
    std::vector<Eigen::VectorXd> control_costs;         /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> total_costs;           /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> cumulative_costs;      /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> probabilities;         /**< [num_dimensions] num_time_steps */

    std::vector<double> full_probabilities;             /**< [num_dimensions] probabilities of full trajectory */
    std::vector<double> full_costs;                     /**< [num_dimensions] costs of full trajectory */

    double importance_weight;                           /**< importance sampling weight */
    double log_likelihood;                              /**< log likelihood of observing this rollout (constant terms ignored) */
    double total_cost;                                  /**< state + control cost */

};


} /* namespace stomp */


#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_UTILS_H_ */
