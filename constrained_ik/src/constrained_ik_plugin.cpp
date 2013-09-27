/*
 * constrained_ik_plugin.cpp
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

#include <constrained_ik/constrained_ik_plugin.h>
#include <constrained_ik/basic_ik.h>
#include <ros/ros.h>

#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>

#include <pluginlib/class_list_macros.h>
//register PR2ArmKinematics as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(constrained_ik::ConstrainedIKPlugin, kinematics::KinematicsBase);

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace constrained_ik
{

typedef basic_ik::Basic_IK Solver;

ConstrainedIKPlugin::ConstrainedIKPlugin():active_(false), dimension_(0)
{
}

bool ConstrainedIKPlugin::isActive()
{
    if(active_)
        return true;
    ROS_ERROR("kinematics not active");
    return false;
}

bool ConstrainedIKPlugin::isActive() const
{
    if(active_)
        return true;
    ROS_ERROR("kinematics not active");
    return false;
}

bool ConstrainedIKPlugin::initialize(const std::string& robot_description,
                                     const std::string& group_name,
                                     const std::string& base_name,
                                     const std::string& tip_name,
                                     double search_discretization)
{
    setValues(robot_description, group_name, base_name, tip_name, search_discretization);

    //get robot data from parameter server
    urdf::Model robot_model;
    if (!robot_model.initParam(robot_description))
    {
        ROS_ERROR_STREAM("Could not load URDF model from " << robot_description);
        active_ = false;
        return false;
    }

    //initialize kinematic solver with robot info
    if (!kin_.init(robot_model, base_frame_, tip_frame_))
    {
        ROS_ERROR("Could not load ik");
        active_ = false;
    }
    else
    {
        dimension_ = kin_.numJoints();
        kin_.getJointNames(joint_names_);
        kin_.getLinkNames(link_names_);
        active_ = true;
    }
    return active_;
}

bool ConstrainedIKPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                        const std::vector<double> &ik_seed_state,
                                        std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const
{
    //check that solver is ready and properly configured
    if(!active_)
    {
        ROS_ERROR("kinematics not active");
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }
    if(ik_seed_state.size() != dimension_)
    {
        ROS_ERROR("ik_seed_state does not have same dimension as solver");
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    //convert input parameters to required types
    KDL::Frame pose_desired;
    tf::poseMsgToKDL(ik_pose, pose_desired);
    Eigen::Affine3d goal;
    tf::transformKDLToEigen(pose_desired, goal);

    Eigen::VectorXd seed(dimension_), joint_angles(dimension_);
    for(size_t ii=0; ii < dimension_; ++ii)
    {
        seed(ii) = ik_seed_state[ii];
    }

    //create solver and initialize with kinematic model
    Solver solver;
    solver.init(kin_);

    //Do IK and report results
    try { solver.calcInvKin(goal, seed, joint_angles); }
    catch (exception &e)
    {
        ROS_ERROR_STREAM("Caught exception from IK: " << e.what());
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }
    solution.resize(dimension_);
    for(size_t ii=0; ii < dimension_; ++ii)
    {
        solution[ii] = joint_angles(ii);
    }
    error_code.val = error_code.SUCCESS;
    return true;
}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            std::vector<double> &solution,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    static IKCallbackFn solution_callback = 0;
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            const std::vector<double> &consistency_limits,
                                            std::vector<double> &solution,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    static IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            std::vector<double> &solution,
                                            const IKCallbackFn &solution_callback,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);
}

bool ConstrainedIKPlugin::searchPositionIK( const geometry_msgs::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            double timeout,
                                            const std::vector<double> &consistency_limits,
                                            std::vector<double> &solution,
                                            const IKCallbackFn &solution_callback,
                                            moveit_msgs::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    if(!active_)
    {
        ROS_ERROR("kinematics not active");
        error_code.val = error_code.FAILURE;
        return false;
    }

    KDL::Frame pose_desired;
    tf::poseMsgToKDL(ik_pose, pose_desired);
    Eigen::Affine3d goal;
    tf::transformKDLToEigen(pose_desired, goal);

    Eigen::VectorXd seed(dimension_), joint_angles;
    for(size_t ii=0; ii < dimension_; ii++)
    {
        seed(ii) = ik_seed_state[ii];
    }

    //Do the IK
    Solver solver;
    solver.init(kin_);
    try { solver.calcInvKin(goal, seed, joint_angles); }
    catch (exception &e)
    {
        ROS_ERROR_STREAM("Caught exception from IK: " << e.what());
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }
    solution.resize(dimension_);
    for(size_t ii=0; ii < dimension_; ++ii)
    {
        solution[ii] = joint_angles(ii);
    }

    // If there is a solution callback registered, check before returning
    if (solution_callback)
    {
        solution_callback(ik_pose, solution, error_code);
        if(error_code.val != error_code.SUCCESS)
            return false;
    }

    // Default: return successfully
    error_code.val = error_code.SUCCESS;
    return true;

}

bool ConstrainedIKPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                        const std::vector<double> &joint_angles,
                                        std::vector<geometry_msgs::Pose> &poses) const
{
    if(!active_)
    {
        ROS_ERROR("kinematics not active");
        return false;
    }

    Eigen::VectorXd jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
        jnt_pos_in(i) = joint_angles[i];
        // ROS_DEBUG("Joint angle: %d %f",i,joint_angles[i]);
    }

    poses.resize(link_names.size());

    std::vector<KDL::Frame> kdl_poses;
    bool valid = kin_.linkTransforms(jnt_pos_in, kdl_poses, link_names);
    for(size_t ii=0; ii < kdl_poses.size(); ++ii)
    {
        tf::poseKDLToMsg(kdl_poses[ii],poses[ii]);
    }
    //TODO remove this printing
    ROS_INFO_STREAM("poses: ");
    for (size_t ii=0; ii<poses.size(); ++ii)
    {
        ROS_INFO_STREAM(poses[ii]);
    }
    return valid;
}

const std::vector<std::string>& ConstrainedIKPlugin::getJointNames() const
{
    isActive();
    return joint_names_;
}

const std::vector<std::string>& ConstrainedIKPlugin::getLinkNames() const
{
    isActive();
    return link_names_;
}


}   //namespace constrained_ik_plugin
