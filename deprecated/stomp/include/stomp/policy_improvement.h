/*********************************************************************
 * Copyright (c) 2010, Willow Garage, Inc.
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

 \file    policy_improvement.h

 \author  Ludovic Righetti, Peter Pastor, Mrinal Kalakrishnan
 \date    May 26, 2010

 **********************************************************************/

#ifndef STOMP_POLICYIMPROVEMENT_H_
#define STOMP_POLICYIMPROVEMENT_H_

// ros includes
#include <ros/ros.h>
#include <Eigen/Core>

// local includes
#include <stomp/covariant_movement_primitive.h>
#include <stomp/multivariate_gaussian.h>

namespace stomp
{

struct Rollout
{
    std::vector<Eigen::VectorXd> parameters_;                       /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_;                            /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_projected_;                  /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> parameters_noise_;                 /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> parameters_noise_projected_;       /**< [num_dimensions] num_parameters */
    Eigen::VectorXd state_costs_;                                   /**< num_time_steps */
    std::vector<Eigen::VectorXd> control_costs_;                    /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> total_costs_;                      /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> cumulative_costs_;                 /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> probabilities_;                    /**< [num_dimensions] num_time_steps */

    std::vector<double> full_probabilities_;    /**< [num_dimensions] probabilities of full trajectory */
    std::vector<double> full_costs_;            /**< [num_dimensions] costs of full trajectory */
    //std::vector<double>

    double importance_weight_;                                      /**< importance sampling weight */
    double log_likelihood_;                                         /**< log likelihood of observing this rollout (constant terms ignored) */
    double total_cost_;                                             /**< state + control cost */
};

class PolicyImprovement
{
public:
    /*!
     * Constructor for the policy improvement class
     */
    PolicyImprovement();

    /*!
     * Destructor
     */
    ~PolicyImprovement();

    /**
     * Initializes the object which is required for all operations to succeed.
     * @param num_rollouts
     * @param num_time_steps
     * @param num_reused_rollouts
     * @param policy
     * @return true on success, false on failure
     */
    bool initialize(const int num_time_steps,
                    const int min_rollouts,
                    const int max_rollouts,
                    const int num_rollouts_per_iteration,
                    boost::shared_ptr<stomp::CovariantMovementPrimitive> policy,
                    double control_cost_weight,
                    bool use_noise_adaptation,
                    const std::vector<double>& noise_min_stddev);

    /**
     * Resets the number of rollouts
     * @param num_rollouts
     * @return
     */
    bool setNumRollouts(const int min_rollouts,
                        const int max_rollouts,
                        const int num_rollouts_per_iteration);

    /**
     * Gets the next set of rollouts. Only "new" rollouts that need to be executed are returned,
     * not rollouts which might be reused from the previous set.
     * @param rollouts_ [num_rollouts][num_dimensions] num_parameters
     * @param noise_variance [num_dimensions] noise standard deviation per dimension
     * @return
     */
    bool getRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, const std::vector<double>& noise_stddev);

    /**
     * Sets the next set of rollouts, possibly after some filtering. Only new rollouts returned by getRollouts() can be set here.
     * @param rollouts_ [num_rollouts][num_dimensions] num_parameters
     */
    bool setRollouts(const std::vector<std::vector<Eigen::VectorXd> >& rollouts);

    /**
     * Gets the rollouts, which have the original parameters + projected noise
     */
    bool getProjectedRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts);

    /**
     * Computes the projected noise after setting the new (possibly filtered) rollouts
     */
    bool computeProjectedNoise();

    /*!
     * Set the costs of each rollout per time-step
     * Only the first "n" rows of the costs matrix is used, where n is the number of rollouts
     * generated by getRollouts(), because some rollouts may be used from previous iterations.
     * Outputs the total cost for each rollout (generated and reused) in rollout_costs_total
     * @param costs
     */
    bool setRolloutCosts(const Eigen::MatrixXd& costs, const double control_cost_weight, std::vector<double>& rollout_costs_total);

    bool setNoiselessRolloutCosts(const Eigen::VectorXd& costs, double& total_cost);

    /**
     * Performs the PI^2 update and provides parameter updates at every time step
     *
     * @param parameter_updates [num_dimensions] num_time_steps x num_parameters
     * @return
     */
    bool improvePolicy(std::vector<Eigen::MatrixXd>& parameter_updates);

    /**
     * Adds extra rollouts to the set of rollouts to be reused
     */
    //bool addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rollout_costs);

    /**
     * Gets weights for the updates for timestep
     * [num_dimensions] num_time_steps
     */
    bool getTimeStepWeights(std::vector<Eigen::VectorXd>& time_step_weights);

    void clearReusedRollouts();

    void getAllRollouts(std::vector<Rollout>& rollouts);
    void getNoiselessRollout(Rollout& rollout);
    void getAdaptedStddevs(std::vector<double>& stddevs);

    void setCostCumulation(bool use_cumulative_costs);

    void resetAdaptiveNoise();

private:

    bool initialized_;

    int num_dimensions_;
    std::vector<int> num_parameters_;
    int num_time_steps_;
    //int num_rollouts_reused_;
    //int num_rollouts_extra_;

    int num_rollouts_;                  /**< Number of rollouts currently available */
    int max_rollouts_;                  /**< Max number of rollouts to use in an update */
    int min_rollouts_;                  /**< Min number of rollouts to use in an update */
    int num_rollouts_per_iteration_;    /**< Number of new rollouts to add per iteration */

    double cost_scaling_h_;

//    bool rollouts_reused_;                                                  /**< Are we reusing rollouts for this iteration? */
//    bool rollouts_reused_next_;                                             /**< Can we reuse rollouts for the next iteration? */
//    bool extra_rollouts_added_;                                             /**< Have the "extra rollouts" been added for use in the next iteration? */
    int num_rollouts_gen_;                                                  /**< How many new rollouts have been generated in this iteration? */

    bool use_cumulative_costs_;                                             /**< Use cumulative costs or state costs? */

    boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;

    std::vector<Eigen::MatrixXd> control_costs_;                            /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<Eigen::MatrixXd> inv_control_costs_;                        /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<Eigen::MatrixXd> projection_matrix_;                        /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<Eigen::MatrixXd> inv_projection_matrix_;                    /**< [num_dimensions] num_parameters x num_parameters */
    double control_cost_weight_;											/**< Sensitivity of the exponential cost described in the
    																			paper*/

    std::vector<Eigen::MatrixXd> basis_functions_;                          /**< [num_dimensions] num_time_steps x num_parameters */

    std::vector<Eigen::VectorXd> parameters_;                               /**< [num_dimensions] num_parameters */

    //std::vector<Rollout> all_rollouts_;
    std::vector<Rollout> rollouts_;
    std::vector<Rollout> reused_rollouts_;
    Rollout noiseless_rollout_;
    bool noiseless_rollout_valid_;
    //std::vector<Rollout> extra_rollouts_;

    std::vector<MultivariateGaussian> noise_generators_;                    /**< objects that generate noise for each dimension */
    std::vector<Eigen::MatrixXd> parameter_updates_;                        /**< [num_dimensions] num_time_steps x num_parameters */
    std::vector<Eigen::VectorXd> time_step_weights_;                        /**< [num_dimensions] num_time_steps: Weights computed for updates per time-step */

    // covariance matrix adaptation variables
    std::vector<double> adapted_stddevs_;
    std::vector<Eigen::MatrixXd> adapted_covariances_;
    //std::vector<Eigen::MatrixXd> adapted_covariance_inverse_;
    bool adapted_covariance_valid_;
    bool use_covariance_matrix_adaptation_;
    bool use_projection_;
    std::vector<double> noise_min_stddev_;

    // temporary variables pre-allocated for efficiency:
    std::vector<Eigen::VectorXd> tmp_noise_;                /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> tmp_parameters_;           /**< [num_dimensions] num_parameters */
    Eigen::VectorXd tmp_max_cost_;                          /**< num_time_steps */
    Eigen::VectorXd tmp_min_cost_;                          /**< num_time_steps */
    Eigen::VectorXd tmp_max_minus_min_cost_;                /**< num_time_steps */
    Eigen::VectorXd tmp_sum_rollout_probabilities_;         /**< num_time_steps */
    std::vector<std::pair<double, int> > rollout_cost_sorter_;  /**< vector used for sorting rollouts by their cost */
    bool preAllocateTempVariables();
    bool preComputeProjectionMatrices();

    bool computeRolloutControlCosts();
    bool computeRolloutCumulativeCosts(std::vector<double>& rollout_costs_total);
    bool computeRolloutProbabilities();
    bool computeParameterUpdates();

    bool computeNoise(Rollout& rollout);
    bool computeProjectedNoise(Rollout& rollout);
    bool computeRolloutControlCosts(Rollout& rollout);
    bool computeRolloutCumulativeCosts(Rollout& rollout);
    bool copyParametersFromPolicy();

    bool generateRollouts(const std::vector<double>& noise_variance);

};

}

#endif /* POLICYIMPROVEMENT_H_ */
