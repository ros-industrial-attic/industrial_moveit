/**
 * @file cartesian_planner.cpp
 * @brief Cartesian path planner for moveit.
 *
 * This class is used to represent a cartesian path planner for moveit.
 * It finds a straight line path between the start and goal position. This
 * planner does not have the inherent ability to avoid collision. It does
 * check if the path created is collision free before it returns a trajectory.
 * If a collision is found it returns an empty trajectory and moveit error.
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
#include <constrained_ik/cartesian_planner.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>


namespace constrained_ik
{
  bool CartesianPlanner::solve(planning_interface::MotionPlanResponse &res)
  {
    ros::WallTime start_time = ros::WallTime::now();
    robot_state::RobotState start_state = planning_scene_->getCurrentState();
    robot_state::RobotState goal_state = start_state;
    robot_state::RobotStatePtr mid_state;
    const robot_model::JointModelGroup *group_model = planning_scene_->getRobotModel()->getJointModelGroup(request_.group_name);
    std::vector<std::string> joint_names = group_model->getActiveJointModelNames();
    std::vector<std::string> link_names = group_model->getLinkModelNames();
    Eigen::Affine3d start_pose = start_state.getFrameTransform(link_names.back());
    Eigen::Affine3d goal_pose;
    std::vector<double> pos(1);
    robot_trajectory::RobotTrajectoryPtr traj(new robot_trajectory::RobotTrajectory(planning_scene_->getRobotModel(), request_.group_name));


    ROS_INFO_STREAM("Cartesian Planning for Group: " << request_.group_name);
    for(unsigned int i = 0; i < request_.goal_constraints[0].joint_constraints.size(); i++)
    {
      pos[0]=request_.goal_constraints[0].joint_constraints[i].position;
      goal_state.setJointPositions(joint_names[i], pos);
    }
    goal_pose = goal_state.getFrameTransform(link_names.back());

    ROS_INFO("Setting Position x from %f to %f", start_pose.translation()(0),goal_pose.translation()(0));
    ROS_INFO("Setting Position y from %f to %f", start_pose.translation()(1),goal_pose.translation()(1));
    ROS_INFO("Setting Position z from %f to %f", start_pose.translation()(2),goal_pose.translation()(2));
    ROS_INFO("Setting Position yaw   from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(0),goal_pose.rotation().eulerAngles(3,2,1)(0));
    ROS_INFO("Setting Position pitch from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(1),goal_pose.rotation().eulerAngles(3,2,1)(1));
    ROS_INFO("Setting Position roll  from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(2),goal_pose.rotation().eulerAngles(3,2,1)(2));

    // Generate Interpolated Cartesian Poses
    std::vector<Eigen::Affine3d> poses = interpolateCartesian(start_pose, goal_pose, params_.getCartesianDiscretizationStep());

    // Generate Cartesian Trajectory
    int steps = poses.size();
    mid_state = robot_model::RobotStatePtr(new robot_model::RobotState(start_state));
    for (int j=0; j<steps; j++)
    {
      if (j!=0)
        mid_state->setFromIK(group_model, poses[j], link_names.back());

      mid_state->update();
      traj->addSuffixWayPoint(mid_state, 0.0);

      mid_state = robot_model::RobotStatePtr(new robot_model::RobotState(*mid_state));
    }

    res.planning_time_ = (ros::WallTime::now() - start_time).toSec();

    // Check if traj is a collision free path
    if (planning_scene_->isPathValid(*traj, request_.group_name))
    {
      ROS_INFO("Cartesian Trajectory is collision free! :)");
      res.trajectory_=traj;
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return true;
    }
    else
    {
      ROS_INFO("Cartesian Trajectory is not collision free. :(");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }
  }

  std::vector<Eigen::Affine3d>
  CartesianPlanner::interpolateCartesian(const Eigen::Affine3d& start,
                                            const Eigen::Affine3d& stop,
                                            double ds) const
  {
    // Required position change
    Eigen::Vector3d delta = (stop.translation() - start.translation());
    Eigen::Vector3d start_pos = start.translation();

    // Calculate number of steps
    unsigned steps = static_cast<unsigned>(delta.norm() / ds) + 1;

    // Step size
    Eigen::Vector3d step = delta / steps;

    // Orientation interpolation
    Eigen::Quaterniond start_q (start.rotation());
    Eigen::Quaterniond stop_q (stop.rotation());
    double slerp_ratio = 1.0 / steps;

    std::vector<Eigen::Affine3d> result;
    result.reserve(steps);
    for (unsigned i = 0; i <= steps; ++i)
    {
      Eigen::Vector3d trans = start_pos + step * i;
      Eigen::Quaterniond q = start_q.slerp(slerp_ratio * i, stop_q);
      Eigen::Affine3d pose (Eigen::Translation3d(trans) * q);
      result.push_back(pose);
    }
    return result;
  }
} //namespace constrained_ik

