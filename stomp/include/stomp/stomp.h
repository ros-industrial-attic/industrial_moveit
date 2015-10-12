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

#ifndef STOMP_POLICY_IMPROVEMENT_LOOP_H_
#define STOMP_POLICY_IMPROVEMENT_LOOP_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <stomp/covariant_movement_primitive.h>
#include <stomp/task.h>
#include <stomp/policy_improvement.h>

namespace stomp
{

struct NoiseCoefficients
{
  std::string group_name;
  std::vector<double> stddev;
  std::vector<double> min_stddev;
  std::vector<double> decay;
};

class STOMP
{
public:
    STOMP();
    virtual ~STOMP();

    //bool initializeAndRunTaskByName(ros::NodeHandle& node_handle, std::string& task_name);

    // task must already be initialized at this point.
    bool initialize(const ros::NodeHandle& node_handle, boost::shared_ptr<Task> task);

    bool runSingleIteration(int iteration_number);
    void clearReusedRollouts();

    bool doGenRollouts(int iteration_number);
    bool doExecuteRollouts(int iteration_number);
    bool doRollouts(int iteration_number);
    bool doUpdate(int iteration_number);
    bool doNoiselessRollout(int iteration_number);

    void getAllRollouts(std::vector<Rollout>& rollouts);
    void getNoiselessRollout(Rollout& rollout);
    void getAdaptedStddevs(std::vector<double>& stddevs);
    void getBestNoiselessParameters(std::vector<Eigen::VectorXd>& parameters, double& cost);

    bool runUntilValid();
    bool runUntilValid(int max_iterations, int iterations_after_collision_free);
    void setCostCumulation(bool use_cumulative_costs);

    void resetAdaptiveNoise();

    void proceed(bool proceed);
    bool getProceed();

private:

    bool initialized_;
    ros::NodeHandle node_handle_;

    int num_threads_;

    int min_rollouts_;
    int max_rollouts_;
    int num_rollouts_per_iteration_;
    int num_time_steps_;
    int num_dimensions_;
    int max_iterations_;
    int max_iterations_after_collision_free_;
    double cost_convergence_; // percentage : 0 < c < 1
    int max_iteration_after_cost_convergence_; // should be less than max_iterations

    bool write_to_file_;
    bool use_noise_adaptation_;
    bool use_openmp_;

    bool proceed_;
    boost::mutex proceed_mutex_;

    boost::shared_ptr<Task> task_;
    boost::shared_ptr<CovariantMovementPrimitive> policy_;

    PolicyImprovement policy_improvement_;

    std::vector<Eigen::VectorXd> best_noiseless_parameters_;
    double best_noiseless_cost_;

    bool last_noiseless_rollout_valid_;

    std::vector<std::vector<Eigen::VectorXd> > rollouts_; /**< [num_rollouts][num_dimensions] num_parameters */
    std::vector<std::vector<Eigen::VectorXd> > projected_rollouts_;
    std::vector<Eigen::MatrixXd> parameter_updates_;
    std::vector<Eigen::VectorXd> parameters_;
    std::vector<Eigen::VectorXd> time_step_weights_;
    Eigen::MatrixXd rollout_costs_;
    std::vector<double> noise_stddev_;
    std::vector<double> noise_decay_;
    std::vector<double> noise_min_stddev_;

    // noise coefficients for each planning group
    std::map<std::string,NoiseCoefficients> noise_coefficients_;
    double control_cost_weight_;

    // temporary variables
    std::vector<Eigen::VectorXd> tmp_rollout_cost_;
    std::vector<Eigen::MatrixXd> tmp_rollout_weighted_features_;

    bool readParameters();

    int policy_iteration_counter_;
    bool readPolicy(const int iteration_number);
    bool writePolicy(const int iteration_number, bool is_rollout = false, int rollout_id = 0);

    //bool writePolicyImprovementStatistics(const policy_improvement_loop::PolicyImprovementStatistics& stats_msg);

};

}

#endif /* POLICY_IMPROVEMENT_LOOP_H_ */
