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

using basic_ik::Basic_IK;

ConstrainedIKPlugin::ConstrainedIKPlugin():active_(false), dimension_(0)
{
}

bool ConstrainedIKPlugin::isActive()
{
    if(active_)
        return true;
    return false;
}

bool ConstrainedIKPlugin::initialize(const std::string& robot_description,
                                     const std::string& group_name,
                                     const std::string& base_name,
                                     const std::string& tip_name,
                                     double search_discretization)
{
    setValues(robot_description, group_name, base_name, tip_name, search_discretization);
    urdf::Model robot_model;
//    std::string xml_string;
//    ros::NodeHandle private_handle("~/"+group_name);
//      dimension_ = 7;
//      while(!loadRobotModel(private_handle,robot_model,xml_string) && private_handle.ok())
//      {
//        ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
//        ros::Duration(0.5).sleep();
//      }

//    ROS_DEBUG("Loading KDL Tree");
    robot_model.initParam(robot_description);
    kin_.init(robot_model, base_frame_, tip_frame_);
    dimension_ = kin_.numJoints();
//      if(!getKDLChain(xml_string,base_frame_,tip_frame_,kdl_chain_))
//      {
//        active_ = false;
//        ROS_ERROR("Could not load kdl tree");
//      }
//      ROS_DEBUG("Advertising services");
//      jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
//      private_handle.param<int>("free_angle",free_angle_,2);
//      pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model, base_frame_,tip_frame_, search_discretization_,free_angle_));
    if(!kin_.checkInitialized())
    {
        ROS_ERROR("Could not load ik");
        active_ = false;
    }
    else
    {
        active_ = true;
        kin_.getJointNames(joint_names_);
        kin_.getLinkNames(link_names_);
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

    Eigen::VectorXd seed;
    Eigen::VectorXd joint_angles(dimension_);
    seed.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
        seed(i) = ik_seed_state[i];
    }

    //Do the IK
//      int ik_valid = pr2_arm_ik_solver_->CartToJnt(jnt_pos_in,
//                                                   pose_desired,
//                                                   jnt_pos_out);
//      if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
    Basic_IK solver;
    solver.init(kin_);
    solver.calcInvKin(goal, seed, joint_angles);
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
        solution[i] = joint_angles(i);
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

    Eigen::VectorXd seed;
    Eigen::VectorXd joint_angles(dimension_);
    seed.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
        seed(i) = ik_seed_state[i];
    }

    //Do the IK
//    int ik_valid;
//    if(consistency_limits.empty())
//    {
//        ik_valid = pr2_arm_ik_solver_->CartToJntSearch(seed,
//                                                        pose_desired,
//                                                        joint_angles,
//                                                        timeout,
//                                                        error_code,
//                      solution_callback ? boost::bind(solution_callback, _1, _2, _3): IKCallbackFn());
//    }
//    else
//    {
//        ik_valid = pr2_arm_ik_solver_->CartToJntSearch(seed,
//                                                        pose_desired,
//                                                        joint_angles,
//                                                        timeout,
//                                                        consistency_limits[free_angle_],
//                                                        error_code,
//                solution_callback ? boost::bind(solution_callback, _1, _2, _3): IKCallbackFn());
//    }

    Basic_IK solver;
    solver.init(kin_);
    solver.calcInvKin(goal, seed, joint_angles);
    if (solution_callback)
    {
        solution.resize(dimension_);
        for(size_t ii=0; ii < dimension_; ++ii)
        {
            solution[ii] = joint_angles(ii);
        }
        solution_callback(ik_pose, solution, error_code);
        if(error_code.val == error_code.SUCCESS)
        {
            return true;
        }
        return false;
    }
    else
    {
        error_code.val = error_code.SUCCESS;
        return true;
    }
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

//    KDL::Frame p_out;
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
    Basic_IK solver;
    solver.init(kin_);
    bool valid = solver.calcAllFwdKin(jnt_pos_in, kdl_poses); //TODO this is not hardened against unordered link_names
    for(unsigned int ii=0; ii < kdl_poses.size(); ++ii)
    {
        tf::poseKDLToMsg(kdl_poses[ii],poses[ii]);
    }
    return valid;
}

const std::vector<std::string>& ConstrainedIKPlugin::getJointNames() const
{
    if(!active_)
    {
        ROS_ERROR("kinematics not active");
    }
//    ROS_INFO("I'm in getJointNames()");
    return joint_names_;
}

const std::vector<std::string>& ConstrainedIKPlugin::getLinkNames() const
{
    if(!active_)
    {
        ROS_ERROR("kinematics not active");
    }
    ROS_INFO("I'm in getLinkNames()");
    return link_names_;
}


}   //namespace constrained_ik_plugin
