/**
 * @file constrained_ik_plugin.h
 * @brief Constrained inverse kinematic plugin for moveit.
 *
 * @author dsolomon
 * @date Sep 15, 2013
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef CONSTRAINED_IK_PLUGIN_H_
#define CONSTRAINED_IK_PLUGIN_H_

#include "constrained_ik/basic_kin.h"
#include "constrained_ik/constrained_ik.h"

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MoveItErrorCodes.h>

namespace constrained_ik
{
  /** @brief This class represents the CLIK IK Solver plugin for moveit. */
  class ConstrainedIKPlugin: public kinematics::KinematicsBase
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstrainedIKPlugin();

    /**
     * @brief Indicates whether a solver is active
     * @return True if currently solving, otherwise false
     */
    virtual bool isActive() const;

    /** @brief See base class for documentation */
    bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                       const std::vector<double> &ik_seed_state,
                       std::vector<double> &solution,
                       moveit_msgs::MoveItErrorCodes &error_code,
                       const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    /** @brief See base class for documentation */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    /** @brief See base class for documentation */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          const std::vector<double> &consistency_limits,
                          std::vector<double> &solution,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    /** @brief See base class for documentation */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    /** @brief See base class for documentation */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          const std::vector<double> &consistency_limits,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    /** @brief See base class for documentation */
    bool getPositionFK(const std::vector<std::string> &link_names,
                       const std::vector<double> &joint_angles,
                       std::vector<geometry_msgs::Pose> &poses) const override;

    /** @brief See base class for documentation */
    bool initialize(const std::string& robot_description,
                    const std::string& group_name,
                    const std::string& base_name,
                    const std::string& tip_name,
                    double search_discretization) override;

    /** @brief Return all the joint names in the order they are used internally */
    const std::vector<std::string>& getJointNames() const override;

    /** @brief Return all the link names in the order they are represented internally */
    const std::vector<std::string>& getLinkNames() const override;

  protected:

    bool active_;                                     /**< Indicates status of the kinematic solver */
    basic_kin::BasicKin kin_;                         /**< Constrained IK kinematics object */
    int dimension_;                                   /**< Number of joints */
    std::vector<std::string> link_names_;             /**< List of link names */
    std::vector<std::string> joint_names_;            /**< list of joint names */
    planning_scene::PlanningScenePtr planning_scene_; /**< Pointer to planning scene which is used for collision queries */
    moveit::core::RobotStatePtr robot_state_;         /**< Robot State Ptr */
    robot_model::RobotModelPtr robot_model_ptr_;      /**< Robot Model Ptr */
    boost::shared_ptr<Constrained_IK> solver_;        /**< Constrained IK Solver */
  };

}   //namespace constrained_ik


#endif /* CONSTRAINED_IK_PLUGIN_H_ */
