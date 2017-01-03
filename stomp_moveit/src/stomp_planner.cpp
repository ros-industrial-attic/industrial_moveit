/**
 * @file stomp_planner.cpp
 * @brief This defines the stomp planner for MoveIt
 *
 * @author Jorge Nicho
 * @date April 4, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
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
#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <stomp_moveit/stomp_planner.h>
#include <class_loader/class_loader.h>
#include <stomp_core/utils.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


static const std::string DESCRIPTION = "STOMP";
static const double TIMEOUT_INTERVAL = 0.05;
static int const IK_ATTEMPTS = 10;
static int const IK_TIMEOUT = 0.05;

/**
 * @brief Parses a XmlRpcValue and populates a StompComfiguration structure.
 * @param config        The XmlRpcValue of stomp configuration parameters
 * @param group         The moveit planning group
 * @param stomp_config  The stomp configuration structure
 * @return True if sucessfully parsed, otherwise false.
 */
bool parseConfig(XmlRpc::XmlRpcValue config,const moveit::core::JointModelGroup* group,stomp_core::StompConfiguration& stomp_config)
{
  using namespace XmlRpc;
  // Set default values for optional config parameters
  stomp_config.control_cost_weight = 0.0;
  stomp_config.initialization_method = 1; // LINEAR_INTERPOLATION
  stomp_config.num_timesteps = 40;
  stomp_config.delta_t = 1.0;
  stomp_config.num_iterations = 50;
  stomp_config.num_iterations_after_valid = 0;
  stomp_config.max_rollouts = 100;
  stomp_config.num_rollouts = 10;
  stomp_config.exponentiated_cost_sensitivity = 10.0;

  // Load optional config parameters if they exist
  if (config.hasMember("control_cost_weight"))
    stomp_config.control_cost_weight = static_cast<double>(config["control_cost_weight"]);

  if (config.hasMember("initialization_method"))
    stomp_config.initialization_method = static_cast<int>(config["initialization_method"]);

  if (config.hasMember("num_timesteps"))
    stomp_config.num_timesteps = static_cast<int>(config["num_timesteps"]);

  if (config.hasMember("delta_t"))
    stomp_config.delta_t = static_cast<double>(config["delta_t"]);

  if (config.hasMember("num_iterations"))
    stomp_config.num_iterations = static_cast<int>(config["num_iterations"]);

  if (config.hasMember("num_iterations_after_valid"))
    stomp_config.num_iterations_after_valid = static_cast<int>(config["num_iterations_after_valid"]);

  if (config.hasMember("max_rollouts"))
    stomp_config.max_rollouts = static_cast<int>(config["max_rollouts"]);

  if (config.hasMember("num_rollouts"))
    stomp_config.num_rollouts = static_cast<int>(config["num_rollouts"]);

  if (config.hasMember("exponentiated_cost_sensitivity"))
    stomp_config.exponentiated_cost_sensitivity = static_cast<int>(config["exponentiated_cost_sensitivity"]);

  // getting number of joints
  stomp_config.num_dimensions = group->getActiveJointModels().size();
  if(stomp_config.num_dimensions == 0)
  {
    ROS_ERROR("Planning Group %s has no active joints",group->getName().c_str());
    return false;
  }

  return true;
}

namespace stomp_moveit
{

StompPlanner::StompPlanner(const std::string& group,const XmlRpc::XmlRpcValue& config,
                           const moveit::core::RobotModelConstPtr& model):
    PlanningContext(DESCRIPTION,group),
    config_(config),
    robot_model_(model),
    ph_(new ros::NodeHandle("~"))
{
  setup();
}

StompPlanner::~StompPlanner()
{
}

void StompPlanner::setup()
{
  if(!getPlanningScene())
  {
    setPlanningScene(planning_scene::PlanningSceneConstPtr(new planning_scene::PlanningScene(robot_model_)));
  }

  // loading parameters
  try
  {
    // creating tasks
    XmlRpc::XmlRpcValue task_config;
    task_config = config_["task"];
    task_.reset(new StompOptimizationTask(robot_model_,group_,task_config));

    if(!robot_model_->hasJointModelGroup(group_))
    {
      std::string msg = "Stomp Planning Group '" + group_ + "' was not found";
      ROS_ERROR("%s",msg.c_str());
      throw std::logic_error(msg);
    }

    // parsing stomp parameters
    if(!config_.hasMember("optimization") || !parseConfig(config_["optimization" ],robot_model_->getJointModelGroup(group_),stomp_config_))
    {
      std::string msg = "Stomp 'optimization' parameter for group '" + group_ + "' failed to load";
      ROS_ERROR("%s", msg.c_str());
      throw std::logic_error(msg);
    }

    stomp_.reset(new stomp_core::Stomp(stomp_config_,task_));
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    throw std::logic_error("Stomp Planner failed to load configuration for group '" + group_+"'; " + e.getMessage());
  }

}

bool StompPlanner::solve(planning_interface::MotionPlanResponse &res)
{
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);

  // construct the compact response from the detailed one
  res.trajectory_ = detailed_res.trajectory_.back();
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = ros::Duration(wd.sec, wd.nsec).toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool StompPlanner::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  using namespace stomp_core;

  // initializing response
  res.description_.resize(1,"");
  res.processing_time_.resize(1);
  res.trajectory_.resize(1);
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  ros::WallTime start_time = ros::WallTime::now();
  bool success = false;

  trajectory_msgs::JointTrajectory trajectory;
  Eigen::MatrixXd parameters;
  bool planning_success;

  // local stomp config copy
  auto config_copy = stomp_config_;

  // look for seed state
  trajectory_msgs::JointTrajectory seed_traj;
  bool seed_provided = extractSeedTrajectory(request_, seed_traj);


  // create timeout timer
  ros::WallDuration allowed_time(request_.allowed_planning_time);
  ROS_WARN_COND(TIMEOUT_INTERVAL > request_.allowed_planning_time,
                "%s allowed planning time %f is less than the minimum planning time value of %f",
                getName().c_str(),request_.allowed_planning_time,TIMEOUT_INTERVAL);
  ros::Timer timeout_timer = ph_->createTimer(ros::Duration(TIMEOUT_INTERVAL), [&](const ros::TimerEvent& evnt)
  {
    if((ros::WallTime::now() - start_time) > allowed_time)
    {
      ROS_ERROR("%s exceeded allowed time of %f , terminating",getName().c_str(),allowed_time.toSec());
      this->terminate();
    }

  },false);

  //  If a seed state was provided, then we need to sanity check it.
  bool use_seed = seed_provided;
  if (seed_provided)
  {
    use_seed = checkSeedTrajectory(seed_traj);
    if (!use_seed)
    {
      ROS_WARN("Seed state rejected by STOMP");
    }
  }

  if (use_seed)
  {
    ROS_INFO("%s Seeding trajectory from MotionPlanRequest",getName().c_str());

    Eigen::MatrixXd initial_parameters;
    jointTrajectorytoParameters(seed_traj, initial_parameters);

    // smooth trajectory
    smoothSeedTrajectory(initial_parameters);

    // updating time step in stomp configuraion
    config_copy.num_timesteps = seed_traj.points.size();

    // setting up up optimization task
    if(!task_->setMotionPlanRequest(planning_scene_, request_, config_copy, res.error_code_))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    stomp_->setConfig(config_copy);
    planning_success = stomp_->solve(initial_parameters, parameters);
  }
  else
  {

    // extracting start and goal
    Eigen::VectorXd start, goal;
    if(!getStartAndGoal(start,goal))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      ROS_ERROR("STOMP failed to get the start and goal positions");
      return false;
    }

    // setting up up optimization task
    if(!task_->setMotionPlanRequest(planning_scene_,request_, config_copy,res.error_code_))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    stomp_->setConfig(config_copy);
    planning_success = stomp_->solve(start,goal,parameters);
  }

  // stopping timer
  timeout_timer.stop();

  // Handle results
  if(planning_success)
  {
    if(!parametersToJointTrajectory(parameters,trajectory))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    // creating request response
    moveit::core::RobotState robot_state(robot_model_);
    moveit::core::robotStateMsgToRobotState(request_.start_state,robot_state);
    res.trajectory_[0]= robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(
        robot_model_,group_));
    res.trajectory_.back()->setRobotTrajectoryMsg( robot_state,trajectory);
  }
  else
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  // checking against planning scene
  if(planning_scene_ && !planning_scene_->isPathValid(*res.trajectory_.back(),group_,true))
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    success = false;
    ROS_ERROR_STREAM("STOMP Trajectory is in collision");
  }

  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.processing_time_[0] = ros::Duration(wd.sec, wd.nsec).toSec();
  ROS_INFO_STREAM("STOMP found a valid path after "<<res.processing_time_[0]<<" seconds");

  return true;
}

bool StompPlanner::parametersToJointTrajectory(const Eigen::MatrixXd& parameters,
                                               trajectory_msgs::JointTrajectory& trajectory)
{
  // filling trajectory joint values
  trajectory.joint_names = robot_model_->getJointModelGroup(group_)->getActiveJointModelNames();
  trajectory.points.clear();
  trajectory.points.resize(parameters.cols());
  std::vector<double> vals(parameters.rows());
  std::vector<double> zeros(parameters.rows(),0.0);
  for(auto t = 0u; t < parameters.cols() ; t++)
  {
    Eigen::VectorXd::Map(&vals[0],vals.size()) = parameters.col(t);
    trajectory.points[t].positions = vals;
    trajectory.points[t].velocities = zeros;
    trajectory.points[t].accelerations = zeros;
    trajectory.points[t].time_from_start = ros::Duration(0.0);
  }

  trajectory_processing::IterativeParabolicTimeParameterization time_generator;
  robot_trajectory::RobotTrajectory traj(robot_model_,group_);
  moveit::core::RobotState robot_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state,robot_state);
  traj.setRobotTrajectoryMsg(robot_state,trajectory);

  // converting to msg
  moveit_msgs::RobotTrajectory robot_traj_msgs;
  if(time_generator.computeTimeStamps(traj,request_.max_velocity_scaling_factor))
  {
    traj.getRobotTrajectoryMsg(robot_traj_msgs);
    trajectory = robot_traj_msgs.joint_trajectory;
  }
  else
  {
    ROS_ERROR("%s Failed to generate timing data",getName().c_str());
    return false;
  }
  return true;
}

bool StompPlanner::jointTrajectorytoParameters(const trajectory_msgs::JointTrajectory& traj, Eigen::MatrixXd& parameters) const
{
  const auto dof = traj.joint_names.size();
  const auto timesteps = traj.points.size();

  Eigen::MatrixXd mat (dof, timesteps);

  for (size_t step = 0; step < timesteps; ++step)
  {
    for (size_t joint = 0; joint < dof; ++joint)
    {
      mat(joint, step) = traj.points[step].positions[joint];
    }
  }

  parameters = mat;
  return true;
}

bool StompPlanner::extractSeedTrajectory(const moveit_msgs::MotionPlanRequest& req, trajectory_msgs::JointTrajectory& seed) const
{
  if (req.trajectory_constraints.constraints.empty())
    return false;

  const auto* joint_group = robot_model_->getJointModelGroup(group_);
  const auto& names = joint_group->getActiveJointModelNames();
  const auto dof = names.size();

  const auto& constraints = req.trajectory_constraints.constraints; // alias to keep names short
  // Test the first point to ensure that it has all of the joints required
  for (size_t i = 0; i < constraints.size(); ++i)
  {
    auto n = constraints[i].joint_constraints.size();
    if (n != dof) // first test to ensure that dimensionality is correct
    {
      ROS_WARN("Seed trajectory index %lu does not have %lu constraints (has %lu instead).", i, dof, n);
      return false;
    }

    trajectory_msgs::JointTrajectoryPoint joint_pt;

    for (size_t j = 0; j < constraints[i].joint_constraints.size(); ++j)
    {
      const auto& c = constraints[i].joint_constraints[j];
      if (c.joint_name != names[j])
      {
        ROS_WARN("Seed trajectory (index %lu, joint %lu) joint name '%s' does not match expected name '%s'",
                 i, j, c.joint_name.c_str(), names[j].c_str());
        return false;
      }
      joint_pt.positions.push_back(c.position);
    }

    seed.points.push_back(joint_pt);
  }

  seed.joint_names = names;
  return true;
}

/**
 * @brief Checks to see if a given robot joint pose, given by names/positions, is within
 * 'max_delta' of the given robot state.
 * @return True if total L1 joint distance is less than max_delta
 */
static bool withinTolerance(const std::vector<std::string>& names, const std::vector<double>& positions,
                            const moveit::core::RobotState& state, const double max_delta)
{
  double dist = 0.0;
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    dist += std::abs(positions[i] - state.getVariablePosition(names[i]));
  }
  return dist <= max_delta;
}

bool StompPlanner::checkSeedTrajectory(trajectory_msgs::JointTrajectory &seed) const
{
  if (seed.points.size() < 2)
    return false;

  moveit::core::RobotState state (robot_model_);
  const auto* group = state.getJointModelGroup(group_);
  const auto& joint_names = group->getActiveJointModelNames();
  const auto& tool_link = group->getLinkModelNames().back();


  // Check to see if the seed is the same DOF
  if (joint_names.size() != seed.joint_names.size())
  {
    return false;
  }

  // We check to see if the start state in the request and the seed state are 'close'
  if (!moveit::core::robotStateMsgToRobotState(request_.start_state, state))
  {
    return false;
  }

  const static double MAX_START_DISTANCE_THRESH = 0.5;

  // Check the start state to see if we're close enough
  auto& seed_start_pos = seed.points.front().positions;
  if (!withinTolerance(seed.joint_names, seed_start_pos, state, MAX_START_DISTANCE_THRESH))
  {
    return false;
  }

  // If we've accepted the start position, then we overwrite this position in the seed trajectory
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    seed_start_pos[i] = state.getVariablePosition(seed.joint_names[i]);
  }

  // Now we need to fix the goals. We start this by initializing our state object to the final
  // position of the seed
  state.setJointGroupPositions(group, seed.points.back().positions);

  // extracting goal joint values
  bool found_goal = false;
  for(const auto& gc : request_.goal_constraints)
  {
    if (gc.joint_constraints.empty())
    {
      // cartesian goal specified - we want to search to the closest point
      const moveit_msgs::PositionConstraint& pos_constraint = gc.position_constraints.front();
      const moveit_msgs::OrientationConstraint& orient_constraint = gc.orientation_constraints.front();

      geometry_msgs::Pose pose;
      pose.position = pos_constraint.constraint_region.primitive_poses[0].position;
      pose.orientation = orient_constraint.orientation;

      if(!state.setFromIK(group, pose, tool_link, IK_ATTEMPTS, IK_TIMEOUT) ||
         !planning_scene_->isStateValid(state))
      {
        ROS_WARN("SEED %s failed calculating ik for cartesian goal pose in the MotionPlanRequest",getName().c_str());
        continue;
      }

      // copying values into goal array
      state.enforceBounds(group);

      const static double MAX_SEED_DISTANCE_AFTER_IK = 0.5;
      if (!withinTolerance(joint_names, seed.points.back().positions, state, MAX_SEED_DISTANCE_AFTER_IK))
      {
        continue;
      }

      for(auto j = 0u; j < joint_names.size(); j++)
      {
        seed.points.back().positions[j] = state.getVariablePosition(joint_names[j]);
      }

      found_goal = true;
      break;
    }
    else
    {
      // Joint goal specified
      auto& end_seed_state = seed.points.back().positions;
      // copying goal values into state
      moveit::core::RobotState test_state (state);
      for(auto j = 0u; j < gc.joint_constraints.size(); j++)
      {
        auto jc = gc.joint_constraints[j];
        test_state.setVariablePosition(jc.joint_name,jc.position);
      }

      if (withinTolerance(seed.joint_names, end_seed_state, test_state, MAX_START_DISTANCE_THRESH))
      {
        for (std::size_t i = 0; i < seed.joint_names.size(); ++i)
        {
          end_seed_state[i] = test_state.getVariablePosition(seed.joint_names[i]);
        }
        found_goal = true;

        break;
      }
    }
  } // end goal loop

  return found_goal;
}

bool StompPlanner::smoothSeedTrajectory(Eigen::MatrixXd &parameters, int polynomial) const
{
  const auto n_rows = parameters.rows();
  const auto n_samples = parameters.cols();

  // We smooth with the assumption that each step is a fixed time interval?
  Eigen::VectorXd x (n_samples);
  for (size_t i = 0; i < n_samples; ++i)
  {
    x(i) = i;
  }

  // We must hold the first and last point to their target positions
  Eigen::VectorXi fixed_points (2);
  fixed_points(0) = 0;
  fixed_points(1) = n_samples - 1;

  // For each row, we replace the current set of joint values with 'smooth' ones built
  // from a polynomial fit to that joint's motion.
  for(size_t i = 0; i < n_rows; ++i)
  {
    Eigen::VectorXd poly_params;
    Eigen::VectorXd new_y = stomp_core::polyFitWithFixedPoints(polynomial, x, parameters.row(i),
                                                               fixed_points, poly_params);

    parameters.row(i) = new_y;
  }

  return true;
}

moveit_msgs::TrajectoryConstraints StompPlanner::encodeSeedTrajectory(const trajectory_msgs::JointTrajectory &seed)
{
  moveit_msgs::TrajectoryConstraints res;

  const auto dof = seed.joint_names.size();

  for (size_t i = 0; i < seed.points.size(); ++i) // for each time step
  {
    moveit_msgs::Constraints c;

    if (seed.points[i].positions.size() != dof)
      throw std::runtime_error("All trajectory position fields must have same dimensions as joint_names");

    for (size_t j = 0; j < dof; ++j) // for each joint
    {
      moveit_msgs::JointConstraint jc;
      jc.joint_name = seed.joint_names[j];
      jc.position = seed.points[i].positions[j];

      c.joint_constraints.push_back(jc);
    }

    res.constraints.push_back(std::move(c));
  }

  return res;
}

bool StompPlanner::getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
  using namespace moveit::core;
  RobotStatePtr state(new RobotState(robot_model_));
  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_);
  std::string tool_link = joint_group->getLinkModelNames().back();
  bool found_goal = false;

  try
  {
    // copying start state
    if(!robotStateMsgToRobotState(request_.start_state,*state))
    {
      ROS_ERROR("%s Failed to extract start state from MotionPlanRequest",getName().c_str());
      return false;
    }

    // copying start joint values
    const std::vector<std::string> joint_names= state->getJointModelGroup(group_)->getActiveJointModelNames();
    start.resize(joint_names.size());
    goal.resize(joint_names.size());
    state->enforceBounds(joint_group);
    for(auto j = 0u; j < joint_names.size(); j++)
    {
      start(j) = state->getVariablePosition(joint_names[j]);
    }

    // check goal constraint
    if(request_.goal_constraints.empty())
    {
      ROS_ERROR("%s A goal constraint was not provided",getName().c_str());
      return false;
    }

    // extracting goal joint values
    for(const auto& gc : request_.goal_constraints)
    {

      //auto& gc = request_.goal_constraints.front();
      if(gc.joint_constraints.empty())
      {
        // solving ik at goal
        const moveit_msgs::PositionConstraint& pos_constraint = gc.position_constraints.front();
        const moveit_msgs::OrientationConstraint& orient_constraint = gc.orientation_constraints.front();

        geometry_msgs::Pose pose;
        pose.position = pos_constraint.constraint_region.primitive_poses[0].position;
        pose.orientation = orient_constraint.orientation;

        if(!state->setFromIK(joint_group,pose,tool_link,IK_ATTEMPTS,IK_TIMEOUT))
        {
          ROS_DEBUG("%s failed calculating ik for cartesian goal pose in the MotionPlanRequest",getName().c_str());
          continue;
        }

        // copying values into goal array
        state->enforceBounds(joint_group);
        for(auto j = 0u; j < joint_names.size(); j++)
        {
          goal(j) = state->getVariablePosition(joint_names[j]);
        }

        found_goal = true;
        break;

      }
      else
      {
        // copying goal values into state
        for(auto j = 0u; j < gc.joint_constraints.size() ; j++)
        {
          auto jc = gc.joint_constraints[j];
          state->setVariablePosition(jc.joint_name,jc.position);
        }

        // copying values into goal array
        state->enforceBounds(joint_group);
        for(auto j = 0u; j < joint_names.size(); j++)
        {
          goal(j) = state->getVariablePosition(joint_names[j]);
        }

        found_goal = true;
        break;

      }
    }

    ROS_ERROR_COND(!found_goal,"%s was unable to retrieve the goal from the MotionPlanRequest",getName().c_str());

  }
  catch(moveit::Exception &e)
  {
    ROS_ERROR("Failure retrieving start or goal state joint values from request %s", e.what());
    return false;
  }

  return found_goal;
}


bool StompPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  // check group
  if(req.group_name != getGroupName())
  {
    ROS_ERROR("STOMP: Unsupported planning group '%s' requested", req.group_name.c_str());
    return false;
  }

  // check for single goal region
  if (req.goal_constraints.size() != 1)
  {
    ROS_ERROR("STOMP: Can only handle a single goal region.");
    return false;
  }

  // check that we have only joint constraints at the goal
  if (req.goal_constraints[0].joint_constraints.size() == 0)
  {
    ROS_ERROR("STOMP: Can only handle joint space goals.");
    return false;
  }

  return true;
}

bool StompPlanner::terminate()
{
  if(stomp_)
  {
    if(!stomp_->cancel())
    {
      ROS_ERROR_STREAM("Failed to interrupt Stomp");
      return false;
    }
  }
  return true;
}

void StompPlanner::clear()
{
  stomp_->clear();
}

bool StompPlanner::getConfigData(ros::NodeHandle &nh, std::map<std::string, XmlRpc::XmlRpcValue> &config, std::string param)
{
  // Create a stomp planner for each group
  XmlRpc::XmlRpcValue stomp_config;
  if(!nh.getParam(param, stomp_config))
  {
    ROS_ERROR("The 'stomp' configuration parameter was not found");
    return false;
  }

  // each element under 'stomp' should be a group name
  std::string group_name;
  try
  {
    for(XmlRpc::XmlRpcValue::iterator v = stomp_config.begin(); v != stomp_config.end(); v++)
    {
      group_name = static_cast<std::string>(v->second["group_name"]);
      config.insert(std::make_pair(group_name, v->second));
    }
    return true;
  }
  catch(XmlRpc::XmlRpcException& e )
  {
    ROS_ERROR("Unable to parse ROS parameter:\n %s",stomp_config.toXml().c_str());
    return false;
  }
}


} /* namespace stomp_moveit_interface */

