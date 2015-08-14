/**
 * @file joint_interpolation_planner.cpp
 * @brief Joint interpolation planner for moveit.
 *
 * This class is used to represent a joint interpolated path planner for
 * moveit.  This planner does not have the inherent ability to avoid
 * collision. It does check if the path created is collision free before it
 * returns a trajectory.  If a collision is found it returns an empty
 * trajectory and moveit error.
 *
 * @author Levi Armstrong
 * @date May 4, 2015
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2015, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <constrained_ik/moveit_interface/joint_interpolation_planner.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>


namespace constrained_ik
{
  bool JointInterpolationPlanner::solve(planning_interface::MotionPlanResponse &res)
  {
    ros::WallTime start_time = ros::WallTime::now();
    robot_model::RobotModelConstPtr rob_model = planning_scene_->getRobotModel();
    robot_state::RobotState start_state(rob_model);
    robot_state::robotStateMsgToRobotState(request_.start_state, start_state);
    robot_state::RobotState goal_state = start_state;
    robot_state::RobotStatePtr mid_state;
    const robot_model::JointModelGroup *group_model = rob_model->getJointModelGroup(request_.group_name);
    std::vector<std::string> joint_names = group_model->getActiveJointModelNames();
    std::vector<std::string> link_names = group_model->getLinkModelNames();
    Eigen::Affine3d goal_pose;
    std::vector<double> pos(1);
    robot_trajectory::RobotTrajectoryPtr traj(new robot_trajectory::RobotTrajectory(rob_model, request_.group_name));


    ROS_INFO_STREAM("Joint Interpolation Planning for Group: " << request_.group_name);

    // if we have path constraints, we prefer interpolating in pose space
    if (!request_.goal_constraints[0].joint_constraints.empty())
    {
      for(unsigned int i = 0; i < request_.goal_constraints[0].joint_constraints.size(); i++)
      {
        pos[0]=request_.goal_constraints[0].joint_constraints[i].position;
        goal_state.setJointPositions(joint_names[i], pos);

        ROS_DEBUG_NAMED("clik","Setting joint %s from %f to position %f", request_.goal_constraints[0].joint_constraints[i].joint_name.c_str(),
            *start_state.getJointPositions(joint_names[i]), request_.goal_constraints[0].joint_constraints[i].position);
      }
    }
    else
    {
      geometry_msgs::Pose pose;
      if (!request_.goal_constraints[0].position_constraints.empty() && !request_.goal_constraints[0].orientation_constraints.empty())
      {
        pose.position = request_.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position;
        pose.orientation = request_.goal_constraints[0].orientation_constraints[0].orientation;
      }
      else if (!request_.goal_constraints[0].position_constraints.empty() && request_.goal_constraints[0].orientation_constraints.empty())
      {
        tf::poseEigenToMsg(start_state.getFrameTransform(link_names.back()), pose);
        pose.position = request_.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position;
      }
      else if (request_.goal_constraints[0].position_constraints.empty() && !request_.goal_constraints[0].orientation_constraints.empty())
      {
        tf::poseEigenToMsg(start_state.getFrameTransform(link_names.back()), pose);
        pose.orientation = request_.goal_constraints[0].orientation_constraints[0].orientation;
      }
      else
      {
        ROS_ERROR("No constraint was passed with request!");
        return false;
      }

      tf::poseMsgToEigen(pose, goal_pose);
      goal_state.setFromIK(group_model, goal_pose, link_names.back());
    }

    // Calculate delta for for moveit interpolation function
    double dt;
    Eigen::VectorXd jv_step;
    Eigen::VectorXd jv_start;
    Eigen::VectorXd delta;

    start_state.copyJointGroupPositions(request_.group_name, jv_start);
    mid_state = robot_model::RobotStatePtr(new robot_model::RobotState(start_state));
    start_state.interpolate(goal_state, 0.1, *mid_state);
    mid_state->copyJointGroupPositions(request_.group_name, jv_step);
    delta = (jv_step - jv_start).cwiseAbs();
    dt = params_.getJointDiscretizationStep()*(0.1/delta.maxCoeff());

    // Generate Path
    int steps = (1.0/dt) + 1;
    dt = 1.0/steps;

    for (int j=0; j<=steps; j++)
    {
      if (j!=steps)
        start_state.interpolate(goal_state, j*dt, *mid_state);
      else
        start_state.interpolate(goal_state, 1, *mid_state);

      traj->addSuffixWayPoint(*mid_state, 0.0);

      if (terminate_)
        break;
    }
    res.planning_time_ = (ros::WallTime::now() - start_time).toSec();

    // Check if planner was terminated
    if (terminate_)
    {
      ROS_INFO("Joint Interpolated Trajectory was terminated!");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }

    // Check if traj is a collision free path
    if (planning_scene_->isPathValid(*traj, request_.group_name))
    {
      ROS_INFO("Joint Interpolated Trajectory is collision free! :)");
      res.trajectory_=traj;
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return true;
    }
    else
    {
      ROS_INFO("Joint Interpolated Trajectory is not collision free. :(");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }
  }
} //namespace constrained_ik
