/*
 * joint_interpolated_planner.cpp
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
#include <constrained_ik/joint_interpolation_planner.h>
#include <ros/ros.h>


namespace constrained_ik
{
  bool JointInterpolationPlanner::solve(planning_interface::MotionPlanResponse &res)
  {
    ros::WallTime start_time = ros::WallTime::now();
    robot_state::RobotState start_state = planning_scene_->getCurrentState();
    robot_state::RobotState goal_state = start_state;
    robot_state::RobotStatePtr mid_state;
    const robot_model::JointModelGroup *group_model = planning_scene_->getRobotModel()->getJointModelGroup(request_.group_name);
    std::vector<std::string> joint_names = group_model->getActiveJointModelNames();
    std::vector<double> pos(1);
    robot_trajectory::RobotTrajectoryPtr traj(new robot_trajectory::RobotTrajectory(planning_scene_->getRobotModel(), request_.group_name));


    ROS_INFO_STREAM("Joint Interpolation Planning for Group: " << request_.group_name);
    for(unsigned int i = 0; i < request_.goal_constraints[0].joint_constraints.size(); i++)
    {
      ROS_INFO_STREAM("Setting joint " << request_.goal_constraints[0].joint_constraints[i].joint_name
                      << " from " << *start_state.getJointPositions(joint_names[i])
                      << " to position " << request_.goal_constraints[0].joint_constraints[i].position);

      pos[0]=request_.goal_constraints[0].joint_constraints[i].position;
      goal_state.setJointPositions(joint_names[i], pos);
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
      mid_state = robot_model::RobotStatePtr(new robot_model::RobotState(start_state));

      if (j!=steps)
        start_state.interpolate(goal_state, j*dt, *mid_state);
      else
        start_state.interpolate(goal_state, 1, *mid_state);

      mid_state->update();
      traj->addSuffixWayPoint(mid_state, 0.0);
    }
    res.planning_time_ = (ros::WallTime::now() - start_time).toSec();

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
