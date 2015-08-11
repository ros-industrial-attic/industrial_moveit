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

#ifndef STOMP_COVARIANT_MOVEMENT_PRIMITIVE_H_
#define STOMP_COVARIANT_MOVEMENT_PRIMITIVE_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <stomp/stomp_utils.h>

namespace stomp
{

class CovariantMovementPrimitive
{
public:
    CovariantMovementPrimitive();
    virtual ~CovariantMovementPrimitive();

    /**
     * Initialize the covariant movement primitive
     *
     * @param num_time_steps Number of time steps
     * @param num_dimensions Number of dimensions
     * @param movement_duration Duration of the entire movement
     * @param derivative_costs[dimension](time_step, derivative)
         Cost of each derivative at each time-step
         Derivative 0 = position, 1 = velocity, 2 = acceleration, 3 = jerk
         Time-steps must include TRAJECTORY_PADDING extra steps before and after the movement to be optimized
     * @param initial_trajectory[dimension](time_step) Initial trajectory to initialize from
         When position costs are include (above), the corresponding cost function penalizes squared error from
         the initial trajectory provided for each time-step.
     * @return
     */
    bool initialize(const int num_time_steps,
                  const int num_dimensions,
                  const double movement_duration,
                  const std::vector<Eigen::MatrixXd>& derivative_costs,
                  const std::vector<Eigen::VectorXd>& initial_trajectory);
    bool setToMinControlCost();
    bool getParametersAll(std::vector<Eigen::VectorXd>& parameters);

    /**
     * Sets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, false on failure
     */
    bool setNumTimeSteps(const int num_time_steps);

    /**
     * Gets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, fase on failure
     */
    bool getNumTimeSteps(int& num_time_steps);

    /**
     * Gets the number of dimensions
     * @param num_dimensions (output) number of dimensions
     * @return true on success, false on failure
     */
    bool getNumDimensions(int& num_dimensions);

    /**
     * Gets the number of policy parameters per dimension
     *
     * @param num_params (output) vector of number of parameters per dimension
     * @return true on success, false on failure
     */
    bool getNumParameters(std::vector<int>& num_params);

    /**
     * Gets the basis functions that multiply the policy parameters in the dynamical system
     * @param basis_function_matrix_array (output) Array of "num_time_steps x num_parameters" matrices, per dimension
     * @return true on success, false on failure
     */
    bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions);

    /**
     * Gets the positive semi-definite matrix of the quadratic control cost
     * The weight of this control cost is provided by the task
     *
     * @param control_cost_matrix (output) Array of square, positive semi-definite matrix: num_params x num_params
     * @return true on success, false on failure
     */
    bool getControlCosts(std::vector<Eigen::MatrixXd>& control_costs);

    bool getInvControlCosts(std::vector<Eigen::MatrixXd>& control_costs);
    /**
     * Update the policy parameters based on the updates per timestep
     * @param updates (input) parameter updates per time-step, num_time_steps x num_parameters
     * @return true on success, false on failure
     */
    bool updateParameters(const std::vector<Eigen::MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights);

    /**
     * Get the policy parameters per dimension
     * @param parameters (output) array of parameter vectors
     * @return true on success, false on failure
     */
    bool getParameters(std::vector<Eigen::VectorXd>& parameters);

    /**
     * Set the policy parameters per dimension
     * @param parameters (input) array of parameter vectors
     * @return true on success, false on failure
     */
    bool setParameters(const std::vector<Eigen::VectorXd>& parameters);

    /**
     * Set the policy parameters per dimension
     * @param parameters (input) array of parameter vectors
     * @return true on success, false on failure
     */
    bool setParametersAll(const std::vector<Eigen::VectorXd>& parameters_all);

    /**
     * Compute the control costs over time, given the control cost matrix per dimension and parameters over time
     * @param control_cost_matrices (input) [num_dimensions] num_parameters x num_parameters: Quadratic control cost matrix (R)
     * @param parameters (input) [num_dimensions][num_time_steps] num_parameters: Parameters over time (can also be theta + projected noise)
     * @param weight (input) constant multiplier for the control costs
     * @param control_costs (output) [num_dimensions] num_time_steps: Control costs over time
     * @return
     */
//    bool computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices, const std::vector<std::vector<Eigen::VectorXd> >& parameters,
//                                     const double weight, std::vector<Eigen::VectorXd>& control_costs);

    bool computeControlCosts(const std::vector<Eigen::VectorXd>& parameters,
                             const std::vector<Eigen::VectorXd>& noise, const double weight, std::vector<Eigen::VectorXd>& control_costs);

    bool computeControlCostGradient(const std::vector<Eigen::VectorXd>& parameters,
                                    const double weight,
                                    std::vector<Eigen::VectorXd>& gradient);

    bool writeToFile(const std::string abs_file_name);

    /**
     * Get the n-th derivative of the trajectory
     * @param derivative_number (1 = vel, 2 = acc, 3 = jerk)
     * @param derivatives - output trajectories [num_vars_free]
     * @return true on success, false on failure
     */
    bool getDerivatives(int derivative_number, std::vector<Eigen::VectorXd>& derivatives) const;

    const Eigen::MatrixXd& getDifferentiationMatrix(int derivative_number) const;

    const std::vector<Eigen::VectorXd>& getMinControlCostParameters() const;

    double getMovementDuration() const;
    double getMovementDt() const;
private:

    std::string file_name_base_;

    int num_time_steps_;
    int num_vars_free_;
    int num_vars_all_;
    int free_vars_start_index_;
    int free_vars_end_index_;
    int num_dimensions_;
    double movement_duration_;
    double movement_dt_;

    std::vector<int> num_parameters_;
    std::vector<Eigen::MatrixXd> derivative_costs_;
    std::vector<Eigen::MatrixXd> derivative_costs_sqrt_;
    std::vector<Eigen::MatrixXd> basis_functions_;
    std::vector<Eigen::MatrixXd> control_costs_;
    std::vector<Eigen::MatrixXd> inv_control_costs_;
    std::vector<Eigen::MatrixXd> control_costs_all_;

    std::vector<Eigen::VectorXd> linear_control_costs_;
    std::vector<double> constant_control_costs_; // to make the control cost not appear negative!

    std::vector<Eigen::VectorXd> parameters_all_;
    std::vector<Eigen::VectorXd> min_control_cost_parameters_all_;
    std::vector<Eigen::VectorXd> min_control_cost_parameters_free_;

    std::vector<Eigen::MatrixXd> differentiation_matrices_;
    void createDifferentiationMatrices();
    bool initializeVariables();
    bool initializeCosts();
    bool initializeBasisFunctions();

    bool computeLinearControlCosts();
    bool computeMinControlCostParameters();
};

// inline functions follow

inline bool CovariantMovementPrimitive::getParameters(std::vector<Eigen::VectorXd>& parameters)
{
    if (int(parameters.size()) != num_dimensions_)
    {
        parameters.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
    }
    for (int d=0; d<num_dimensions_; ++d)
    {
        parameters[d] = parameters_all_[d].segment(free_vars_start_index_, num_vars_free_);
    }
    return true;
}

inline bool CovariantMovementPrimitive::getParametersAll(std::vector<Eigen::VectorXd>& parameters)
{
    parameters = parameters_all_;
    return true;
}

inline const std::vector<Eigen::VectorXd>& CovariantMovementPrimitive::getMinControlCostParameters() const
{
  return min_control_cost_parameters_free_;
}

inline bool CovariantMovementPrimitive::setParameters(const std::vector<Eigen::VectorXd>& parameters)
{
    ROS_ASSERT(int(parameters.size()) == num_dimensions_);
    for (int d=0; d<num_dimensions_; ++d)
    {
        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) = parameters[d];
    }
    return true;
}

inline bool CovariantMovementPrimitive::setParametersAll(const std::vector<Eigen::VectorXd>& parameters_all)
{
  ROS_ASSERT(int(parameters_all.size()) == num_dimensions_);
  parameters_all_ = parameters_all;
  return true;
}

inline bool CovariantMovementPrimitive::getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions)
{
    basis_functions = basis_functions_;
    return true;
}

inline bool CovariantMovementPrimitive::getControlCosts(std::vector<Eigen::MatrixXd>& control_costs)
{
    control_costs = control_costs_;
    return true;
}

inline bool CovariantMovementPrimitive::getInvControlCosts(std::vector<Eigen::MatrixXd>& inv_control_costs)
{
  inv_control_costs = inv_control_costs_;
  return true;
}

inline bool CovariantMovementPrimitive::getNumTimeSteps(int& num_time_steps)
{
    num_time_steps = num_time_steps_;
    return true;
}

inline bool CovariantMovementPrimitive::getNumDimensions(int& num_dimensions)
{
    num_dimensions = num_dimensions_;
    return true;
}

inline bool CovariantMovementPrimitive::getNumParameters(std::vector<int>& num_params)
{
    num_params = num_parameters_;
    return true;
}

inline bool CovariantMovementPrimitive::setNumTimeSteps(const int num_time_steps)
{
    ROS_ASSERT_MSG(num_time_steps_ == num_time_steps, "%d != %d", num_time_steps_, num_time_steps);
    return true;
}


}

#endif /* STOMP_COVARIANT_MOVEMENT_PRIMITIVE_H_ */
