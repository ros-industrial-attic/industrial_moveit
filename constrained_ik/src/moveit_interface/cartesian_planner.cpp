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
#include <constrained_ik/moveit_interface/cartesian_planner.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <constrained_ik/basic_kin.h>

namespace constrained_ik
{
  CartesianPlanner::CartesianPlanner(const std::string &name, const std::string &group, const ros::NodeHandle &nh) :
    planning_interface::PlanningContext(name, group),
    terminate_(false),
    robot_description_("robot_description")
  {
    resetPlannerConfiguration();

    // TODO: Defer construction of the constrained_ik solver until solve is called
    solver_.reset(new Constrained_IK(nh));
    std::string constraint_param = "constrained_ik_solver/" + getGroupName() + "/constraints";
    // TODO: We want to load our default set of cost functions here and store them for use
    // when solve() is called.
    ROS_INFO("Load from params");
//    solver_->addConstraintsFromParamServer(constraint_param);

    ROS_INFO("Load into defaults");
    default_constraints_ = solver_->loadConstraintsFromParamServer(constraint_param);
  }

  bool CartesianPlanner::initializeSolver()
  {
    basic_kin::BasicKin kin;
    if (!kin.init(robot_model_->getJointModelGroup(getGroupName())))
    {
      ROS_ERROR("Cartesian planner could not load solver for move_group %s", getGroupName().c_str());
      return false;
    }

    solver_->init(kin);
    return true;
  }

  void CartesianPlanner::setPlannerConfiguration(double translational_discretization_step, double orientational_discretization_step, bool debug_mode)
  {
    if (translational_discretization_step > 0)
      translational_discretization_step_ = translational_discretization_step;
    else
      ROS_WARN("Cartesian Planner translational discretization step must be greater than zero.");

    if (orientational_discretization_step > 0)
      orientational_discretization_step_ = orientational_discretization_step;
    else
      ROS_WARN("Cartesian Planner orientational discretization step must be greater than zero.");

    debug_mode_ = debug_mode;
  }

  void CartesianPlanner::resetPlannerConfiguration()
  {
    translational_discretization_step_ = DEFAULT_TRANSLATIONAL_DISCRETIZATION_STEP;
    orientational_discretization_step_ = DEFAULT_ORIENTATIONAL_DISCRETIZATION_STEP;
    debug_mode_ = false;
  }

  void CartesianPlanner::setSolverConfiguration(const ConstrainedIKConfiguration &config)
  {
    solver_->setSolverConfiguration(config);
  }

  void CartesianPlanner::resetSolverConfiguration()
  {
    solver_->loadDefaultSolverConfiguration();
  }


  static double constraintTolerance(const moveit_msgs::PositionConstraint& pos_constraint)
  {
    double tolerance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < pos_constraint.constraint_region.primitives.size(); ++i)
    {
      const shape_msgs::SolidPrimitive& p = pos_constraint.constraint_region.primitives[i];

      double this_tolerance = std::numeric_limits<double>::max();
      if (p.type == shape_msgs::SolidPrimitive::BOX)
      {
        if (p.dimensions.size() != 3) continue;
        this_tolerance = std::min(std::min(p.dimensions[0], p.dimensions[1]), p.dimensions[2]) / 2.0;
      }

      if (p.type == shape_msgs::SolidPrimitive::SPHERE)
      {
        if (p.dimensions.size() != 1) continue;
        this_tolerance = p.dimensions[0];
      }

      if (this_tolerance != std::numeric_limits<double>::max())
      {
        tolerance = std::min(tolerance, this_tolerance);
      }
    }
    return tolerance;
  }

  static double constraintTolerance(const moveit_msgs::OrientationConstraint& ori_constraint)
  {
    double tolerance = std::min(std::min(ori_constraint.absolute_x_axis_tolerance,
                                         ori_constraint.absolute_y_axis_tolerance),
                                ori_constraint.absolute_z_axis_tolerance);
    return tolerance;
  }

  template<typename Constraint>
  static Constraint mostRestrictive(const std::vector<Constraint>& cs)
  {
    assert(cs.size() > 0);

    double min_tolerance = std::numeric_limits<double>::max();
    unsigned min_index = 0;

    for (unsigned i = 0; i < cs.size(); ++i)
    {
      double this_tol = constraintTolerance(cs[i]);
      if (this_tol < min_tolerance)
      {
        min_tolerance = min_tolerance;
        min_index = i;
      }
    }

    return cs[min_index];
  }

  std::vector<std::pair<bool, Constraint*> > CartesianPlanner::resolveConstraints() const
  {
    // This function attempts to sanely combine goal constraints given by the user with constraints
    // specified as "default" through the parameter server / YAML file interfaces.

    // This planner currently only uses goal constraints and ignores path constraints: the path is specified implictly.
    // With high DOF, we could optimize across several goals but our primary purpose here is low DOF industrial bots.

    // User requested goal constraints take precedence over default ones

    // Warn if user specified any path constraints
    if (request_.path_constraints.joint_constraints.size() > 0 ||
        request_.path_constraints.position_constraints.size() > 0 ||
        request_.path_constraints.orientation_constraints.size() > 0 ||
        request_.path_constraints.visibility_constraints.size() > 0)
    {
      ROS_WARN("Cartesian Planner does not (currently) support path constraints.");
    }

    // First, let's see if the user specifies position
    std::vector<moveit_msgs::PositionConstraint> position_constraints;
    for (std::size_t i = 0; i < request_.goal_constraints.size(); ++i)
    {
      if (request_.goal_constraints[i].position_constraints.size() > 0)
      {
        position_constraints.insert(position_constraints.end(),
                                    request_.goal_constraints[i].position_constraints.begin(),
                                    request_.goal_constraints[i].position_constraints.end());
      }
    }
    const bool user_specifies_position = position_constraints.size() > 0;

    // Next lets look at orientation
    std::vector<moveit_msgs::OrientationConstraint> orientation_constraints;
    for (std::size_t i = 0; i < request_.goal_constraints.size(); ++i)
    {
      if (request_.goal_constraints[i].orientation_constraints.size() > 0)
      {
        orientation_constraints.insert(orientation_constraints.end(),
                                    request_.goal_constraints[i].orientation_constraints.begin(),
                                    request_.goal_constraints[i].orientation_constraints.end());
      }
    }
    const bool user_specifies_orientation = orientation_constraints.size() > 0;

    // If the user has specified position or orientation, we have to choose the best one...
    moveit_msgs::PositionConstraint best_position_constraint;
    moveit_msgs::OrientationConstraint best_orientation_constraint;

    if (user_specifies_position)
    {
      best_position_constraint = mostRestrictive(position_constraints);
    }

    if (user_specifies_orientation)
    {
      best_orientation_constraint = mostRestrictive(orientation_constraints);
    }


    std::vector<std::pair<bool, Constraint*> > result;

    // Create a position constraint, if required
    Constraint* user_pos_constraint = nullptr;
    if (user_specifies_position)
    {
      user_pos_constraint = createConstraint(best_position_constraint);
      result.push_back(std::make_pair(true, user_pos_constraint));
    }

    // Create a orientation constraint, if required
    Constraint* user_ori_constraint = nullptr;
    if (user_specifies_orientation)
    {
      user_ori_constraint = createConstraint(best_orientation_constraint);
      result.push_back(std::make_pair(true, user_ori_constraint));
    }


    // Now we need to remove constraints from the default set as necessary
    std::vector<Constraint*> to_delete;
    for (std::size_t i = 0; i < default_constraints_.size(); ++i)
    {
      Constraint* c = default_constraints_[i].second;
      if ( (c->constraintType() & Constraint::TYPE_POSITION) && user_specifies_position)
      {
        to_delete.push_back(c);
        continue;
      }

      if ( (c->constraintType() & Constraint::TYPE_ORIENTATION) && user_specifies_orientation)
      {
        to_delete.push_back(c);
        continue;
      }

      result.push_back(std::make_pair(default_constraints_[i].first, c));
    }

    // Cleanup constraints that were loaded but not used - they will not be de-allocated
    for (std::size_t i = 0; i < to_delete.size(); ++i)
    {
      delete to_delete[i];
    }

    return result;
  }

  Constraint* CartesianPlanner::createConstraint(const moveit_msgs::PositionConstraint& pos_constraint) const
  {
    Constraint* user_pos_constraint = nullptr;

    boost::shared_ptr<pluginlib::ClassLoader<constrained_ik::Constraint> > constraint_loader;
    constraint_loader.reset(
          new pluginlib::ClassLoader<constrained_ik::Constraint>("constrained_ik", "constrained_ik::Constraint"));

    user_pos_constraint = constraint_loader->createUnmanagedInstance("constrained_ik/GoalPosition");

    XmlRpc::XmlRpcValue params;

    double tolerance = constraintTolerance(pos_constraint);
    if (tolerance == std::numeric_limits<double>::max() || tolerance == 0.0)
      tolerance = 0.001;

    double weight = pos_constraint.weight != 0.0 ? pos_constraint.weight : 1.0;

    params["position_tolerance"] = tolerance;
    params["weights"].setSize(3);
    params["weights"][0] = weight;
    params["weights"][1] = weight;
    params["weights"][2] = weight;
    params["debug"] = XmlRpc::XmlRpcValue(false);
    user_pos_constraint->loadParameters(params);

    return user_pos_constraint;
 }

  Constraint* CartesianPlanner::createConstraint(const moveit_msgs::OrientationConstraint& orient_constraint) const
  {
    Constraint* user_ori_constraint = nullptr;

    boost::shared_ptr<pluginlib::ClassLoader<constrained_ik::Constraint> > constraint_loader;
    constraint_loader.reset(
          new pluginlib::ClassLoader<constrained_ik::Constraint>("constrained_ik", "constrained_ik::Constraint"));

    user_ori_constraint = constraint_loader->createUnmanagedInstance("constrained_ik/GoalToolOrientation");

    double tolerance = constraintTolerance(orient_constraint);
    tolerance = std::max(tolerance, 0.0001);

    // Determine weight
    double weight = orient_constraint.weight != 0.0 ? orient_constraint.weight : 1.0;

    XmlRpc::XmlRpcValue params;
    params["orientation_tolerance"] = tolerance; // TODO: comes from user
    params["weights"].setSize(3);
    params["weights"][0] = weight;
    params["weights"][1] = weight;
    params["weights"][2] = weight;
    params["debug"] = XmlRpc::XmlRpcValue(false);
    user_ori_constraint->loadParameters(params);

    return user_ori_constraint;
  }

  bool CartesianPlanner::solve(planning_interface::MotionPlanDetailedResponse &res)
  {
    planning_interface::MotionPlanResponse response;
    bool success = solve(response);

    // construct the compact response from the detailed one
    res.trajectory_.push_back(response.trajectory_);
    res.processing_time_.push_back(response.planning_time_);
    res.description_.push_back("Cartesian Constrained IK Planner");
    res.error_code_ = response.error_code_;

    return success;
  }

  bool CartesianPlanner::solve(planning_interface::MotionPlanResponse &res)
  {
    ros::WallTime start_time = ros::WallTime::now();
    robot_state::RobotStatePtr mid_state;
    std::vector<std::string> joint_names, link_names;
    Eigen::Affine3d start_pose, goal_pose;

    robot_model_ = planning_scene_->getRobotModel();

    //  TODO: Create our solver here - we want to generate an intelligent union of the constraints
    // specified in the parameter file and the constraints specified in the moveit request
    std::vector<std::pair<bool, Constraint*> > solver_constraints = resolveConstraints();
    for (std::size_t i = 0; i < solver_constraints.size(); ++i)
    {
      if (solver_constraints[i].first)
        solver_->addConstraint(solver_constraints[i].second, constraint_types::Primary);
      else
        solver_->addConstraint(solver_constraints[i].second, constraint_types::Auxiliary);
    }

    // Load solver if not already loaded
    if(!solver_->isInitialized())
      if (!initializeSolver())
        return false;

    robot_state::RobotState start_state(robot_model_);
    robot_state::robotStateMsgToRobotState(request_.start_state, start_state);
    robot_state::RobotState goal_state = start_state;
    robot_trajectory::RobotTrajectoryPtr traj(new robot_trajectory::RobotTrajectory(robot_model_, request_.group_name));
    const robot_model::JointModelGroup *jmg = robot_model_->getJointModelGroup(request_.group_name);

    joint_names = jmg->getActiveJointModelNames();
    link_names = jmg->getLinkModelNames();
    start_pose = start_state.getGlobalLinkTransform(link_names.back());

    ROS_INFO_STREAM("Cartesian Planning for Group: " << request_.group_name);
    // if we have path constraints, we prefer interpolating in pose space
    if (!request_.goal_constraints[0].joint_constraints.empty())
    {
      std::map<std::string, double> goal_joint_map;
      for (size_t i=0; i<request_.goal_constraints[0].joint_constraints.size(); ++i)
      {
        goal_joint_map[request_.goal_constraints[0].joint_constraints[i].joint_name] =
            request_.goal_constraints[0].joint_constraints[i].position;
      }
      goal_state.setVariablePositions(goal_joint_map);
      goal_state.update();
      goal_pose = goal_state.getGlobalLinkTransform(link_names.back());
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
        tf::poseEigenToMsg(start_state.getGlobalLinkTransform(link_names.back()), pose);
        pose.position = request_.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position;
      }
      else if (request_.goal_constraints[0].position_constraints.empty() && !request_.goal_constraints[0].orientation_constraints.empty())
      {
        tf::poseEigenToMsg(start_state.getGlobalLinkTransform(link_names.back()), pose);
        pose.orientation = request_.goal_constraints[0].orientation_constraints[0].orientation;
      }
      else
      {
        ROS_ERROR("No constraint was passed with request!");
        return false;
      }
      tf::poseMsgToEigen(pose, goal_pose);
    }

    ROS_DEBUG_NAMED("clik", "Setting Position x from %f to %f", start_pose.translation()(0),goal_pose.translation()(0));
    ROS_DEBUG_NAMED("clik", "Setting Position y from %f to %f", start_pose.translation()(1),goal_pose.translation()(1));
    ROS_DEBUG_NAMED("clik", "Setting Position z from %f to %f", start_pose.translation()(2),goal_pose.translation()(2));
    ROS_DEBUG_NAMED("clik", "Setting Position yaw   from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(0),goal_pose.rotation().eulerAngles(3,2,1)(0));
    ROS_DEBUG_NAMED("clik", "Setting Position pitch from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(1),goal_pose.rotation().eulerAngles(3,2,1)(1));
    ROS_DEBUG_NAMED("clik", "Setting Position roll  from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(2),goal_pose.rotation().eulerAngles(3,2,1)(2));

    // Generate Interpolated Cartesian Poses
    Eigen::Affine3d world_to_base = start_state.getGlobalLinkTransform(solver_->getKin().getRobotBaseLinkName()).inverse();
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > poses = interpolateCartesian(
        world_to_base*start_pose, world_to_base*goal_pose, translational_discretization_step_, orientational_discretization_step_);

    // Generate Cartesian Trajectory
    int steps = poses.size();
    Eigen::VectorXd start_joints, joint_angles;
    bool found_ik;
    mid_state = robot_model::RobotStatePtr(new robot_model::RobotState(start_state));
    for (int j=0; j<steps; j++)
    {
      if (j!=0)
      {
        mid_state->copyJointGroupPositions(request_.group_name, start_joints);

        //Do IK and report results
        try
        {
          found_ik = solver_->calcInvKin(poses[j], start_joints, planning_scene_, joint_angles);
          mid_state->setJointGroupPositions(request_.group_name, joint_angles);
          mid_state->update();
        }
        catch (std::exception &e)
        {
          found_ik = false;
          ROS_ERROR_STREAM("Caught exception from IK: " << e.what());
        }

        if (!found_ik || planning_scene_->isStateColliding(*mid_state, request_.group_name))
        {
          if (!debug_mode_)
          {
            ROS_INFO("Cartesian planner was unable to find a valid solution. :(");
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
            return false;
          }
          else
          {
            break;
          }
        }
        
        
      }
      traj->addSuffixWayPoint(*mid_state, 0.0);

      if (terminate_)
        break;
      
      res.planning_time_ = (ros::WallTime::now() - start_time).toSec();
      if (res.planning_time_ > request_.allowed_planning_time)
      {
        ROS_INFO("Cartesian planner was unable to find solution in allowed time. :(");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
        return false;
      }
    }

    // Check if planner was terminated
    if (terminate_)
    {
      ROS_INFO("Cartesian Trajectory was terminated!");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }

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

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >
  CartesianPlanner::interpolateCartesian(const Eigen::Affine3d& start,
                                            const Eigen::Affine3d& stop,
                                            double ds, double dt) const
  {
    // Required position change
    Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
    Eigen::Vector3d start_pos = start.translation();
    Eigen::Affine3d stop_prime = start.inverse()*stop; //This the stop pose represented in the start pose coordinate system
    Eigen::AngleAxisd delta_rotation(stop_prime.rotation());
    //delta_rotation.fromRotationMatrix(stop_prime.rotation())


    // Calculate number of steps
    unsigned steps_translation = static_cast<unsigned>(delta_translation.norm() / ds) + 1;
    unsigned steps_rotation = static_cast<unsigned>(delta_rotation.angle() / dt) + 1;
    unsigned steps = std::max(steps_translation, steps_rotation);

    // Step size
    Eigen::Vector3d step = delta_translation / steps;

    // Orientation interpolation
    Eigen::Quaterniond start_q (start.rotation());
    Eigen::Quaterniond stop_q (stop.rotation());
    double slerp_ratio = 1.0 / steps;

    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > result;
    Eigen::Vector3d trans;
    Eigen::Quaterniond q;
    Eigen::Affine3d pose;
    result.reserve(steps+1);
    for (unsigned i = 0; i <= steps; ++i)
    {
      trans = start_pos + step * i;
      q = start_q.slerp(slerp_ratio * i, stop_q);
      pose = (Eigen::Translation3d(trans) * q);
      result.push_back(pose);
    }
    return result;
  }
} //namespace constrained_ik

