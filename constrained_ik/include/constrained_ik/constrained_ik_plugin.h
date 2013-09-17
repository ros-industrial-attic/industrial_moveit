/*
 * constrained_ik_plugin.h
 *
 *  Created on: Sep 15, 2013
 *      Author: dsolomon
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

#ifndef CONSTRAINED_IK_PLUGIN_H_
#define CONSTRAINED_IK_PLUGIN_H_

#include <ros/ros.h>
#include <constrained_ik/basic_ik.h>

#include <moveit/kinematics_base/kinematics_base.h>

#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

//TODO delete these if possible
#include <kdl/chainfksolverpos_recursive.hpp>

namespace constrained_ik
{
class ConstrainedIKPlugin: public kinematics::KinematicsBase
{
public:
    ConstrainedIKPlugin();

    bool isActive();

    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                               const std::vector<double> &ik_seed_state,
                               std::vector<double> &solution,
                               moveit_msgs::MoveItErrorCodes &error_code,
                               const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool getPositionFK(const std::vector<std::string> &link_names,
                               const std::vector<double> &joint_angles,
                               std::vector<geometry_msgs::Pose> &poses) const;

    /**
  * @brief Initialization function for the kinematics
  * @return True if initialization was successful, false otherwise
  */
    virtual bool initialize(const std::string& robot_description,
                            const std::string& group_name,
                            const std::string& base_name,
                            const std::string& tip_name,
                            double search_discretization);

    /**
  * @brief Return all the joint names in the order they are used internally
  */
    const std::vector<std::string>& getJointNames() const;

    /**
  * @brief Return all the link names in the order they are represented internally
  */
    const std::vector<std::string>& getLinkNames() const;

  protected:

    bool active_;
//    int free_angle_;
//    urdf::Model robot_model_;
//    ros::NodeHandle node_handle_, root_handle_;
//    boost::shared_ptr<basic_ik::Basic_IK> solver_;   //TODO does this have to be shrdptr, and can it directly reference included solver?
//    basic_ik::Basic_IK solver_;
    basic_kin::BasicKin kin_;
    //ros::ServiceServer ik_service_,fk_service_,ik_solver_info_service_,fk_solver_info_service_;
    //tf::TransformListener tf_;
    std::string root_name_;
    int dimension_;
    std::vector<std::string> link_names_, joint_names_;
//    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;
//    KDL::Chain kdl_chain_;
//    moveit_msgs::KinematicSolverInfo ik_solver_info_, fk_solver_info_;

};

}   //namespace constrained_ik


#endif /* CONSTRAINED_IK_PLUGIN_H_ */
