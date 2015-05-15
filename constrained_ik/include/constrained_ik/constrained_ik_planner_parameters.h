/*
 * constrained_ik_planner_parameters.h
 *
 *  Created on: May 4, 2015
 *      Author: Levi Armstrong
 */
/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CONSTRAINED_IK_PLANNER_PARAMETERS_H
#define CONSTRAINED_IK_PLANNER_PARAMETERS_H
#include <ros/ros.h>

namespace constrained_ik
{
  /**
   * @brief This class represents the parameters used by each of the CLIK
   * planners.
   */
  class CLIKParameters
  {
  public:
    CLIKParameters(){ setDefaultValues(); }
    virtual ~CLIKParameters(){}

    /**
     * @brief Gets the joint discretization step parameter.
     * @return a double.
     */
    inline double getJointDiscretizationStep() const { return joint_discretization_step_; }

    /**
     * @brief Sets the joint discretization step parameter.
     * @param value a double argument.
     */
    inline void setJointDiscretizationStep(double value) { joint_discretization_step_ = value; }

    /**
     * @brief Gets the cartesian discretization step parameter.
     * @return a double.
     */
    inline double getCartesianDiscretizationStep() const { return cartesian_discretization_step_; }

    /**
     * @brief Sets the cartesian discretization step parameter.
     * @param value a double argument.
     */
    inline void setCartesianDiscretizationStep(double value) { cartesian_discretization_step_ = value; }

    /**
     * @brief Resets the paramets to there default values.
     */
    inline void reset() { setDefaultValues(); }

    /**
     * @brief Prints the parameters to the console.
     */
    inline void print() const
    {
      ROS_INFO("joint_discretization_step = %f", joint_discretization_step_);
      ROS_INFO("cartesian_discretization_step = %f", cartesian_discretization_step_);
    }

    /**
     * @brief Loads parameters from ROS parameter server.
     * @param nh ROS node handle
     */
    inline void load(ros::NodeHandle nh)
    {
      ROS_INFO("CLIK Parameters Loaded");
      nh.param("joint_discretization_step", joint_discretization_step_, getJointDiscretizationStep());
      nh.param("cartesian_discretization_step", cartesian_discretization_step_, getCartesianDiscretizationStep());
    }

  private:
    /**
     * @brief Sets the default parameter values.
     */
    inline void setDefaultValues()
    {
      joint_discretization_step_ = 0.02;
      cartesian_discretization_step_ = 0.01;
    }

  protected:
    /** This represents the maximum discretization step allowed in joint
     * space during the joint interpolated path planning.
     */
    double joint_discretization_step_;

    /** This represents the maximum discretization step allowed in cartesian
     * space during the cartesian path planning
     */
    double cartesian_discretization_step_;

  };

} //namespace constrained_ik

#endif // CONSTRAINED_IK_PLANNER_PARAMETERS_H
