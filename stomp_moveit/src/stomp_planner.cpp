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
#include <stomp_moveit/utils/kinematics.h>
#include <stomp_moveit/utils/polynomial.h>

#include <trac_ik/trac_ik.hpp>
#include <stomp_moveit/rosconsolecolours.h>
#include <moveit_msgs/DisplayTrajectory.h>


static const std::string DESCRIPTION = "STOMP";
static const double TIMEOUT_INTERVAL = 0.05;
static int const IK_ATTEMPTS = 10;
static int const IK_TIMEOUT = 0.05;
const static double MAX_START_DISTANCE_THRESH = 0.5;

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

    publish_seed_trajectory_ = config_.hasMember("publish_seed_trajectory") and config_["publish_seed_trajectory"];

    if(publish_seed_trajectory_)
    {
      ROS_INFO_STREAM("Publishing seed trajectory on " << ph_->getNamespace() << "/seed_trajectory");
      seed_trajectory_publisher_ = ph_->advertise<moveit_msgs::DisplayTrajectory>("seed_trajectory", 1);
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
  if(success)
  {
    res.trajectory_ = detailed_res.trajectory_.back();
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = ros::Duration(wd.sec, wd.nsec).toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

void publishTrajectory(const trajectory_msgs::JointTrajectory& traj, const ros::Publisher& pub)
{
  moveit_msgs::DisplayTrajectory disp_traj;
  disp_traj.trajectory.resize(1);
  disp_traj.trajectory.front().joint_trajectory = traj;
  pub.publish(disp_traj);
  ROS_YELLOW_STREAM("Publishing seed trajectory...");
}

bool StompPlanner::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  using namespace stomp_core;
  using namespace utils::kinematics;

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

  // look for seed trajectory
  Eigen::MatrixXd initial_parameters;
  bool use_seed = getSeedParameters(initial_parameters);


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

  ROS_YELLOW_STREAM("***********NOW " << (use_seed?"GOING":"NOT GOING") << " INTO SEEEDED MODE**********");

  if (use_seed)
  {
    ROS_INFO("%s Seeding trajectory from MotionPlanRequest",getName().c_str());
    ROS_YELLOW_STREAM("Seed shape is (" << initial_parameters.rows() << "," << initial_parameters.cols() << ")");
    if(isCartesianSeed())
    {
      moveit::core::RobotStatePtr state(new moveit::core::RobotState(robot_model_));
      const std::vector<std::string> joint_names= state->getJointModelGroup(group_)->getActiveJointModelNames();
      request_.start_state = robotStateFromEigen(initial_parameters.leftCols(1), joint_names, state->getVariableNames());
      request_.goal_constraints.clear();
      request_.goal_constraints.push_back(jointConstraintsFromEigen(initial_parameters.rightCols(1), joint_names));
    }

    // updating time step in stomp configuraion
    config_copy.num_timesteps = initial_parameters.cols();

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
    trajectory.header.seq=stomp_->getNumIterations(); //number of iterations
    ROS_INFO_STREAM("Bruno this is your number of luck "<<trajectory.header.seq);
    trajectory.points[0].effort.resize(trajectory.points[0].positions.size());
    trajectory.points[0].effort[0]=stomp_->getNumIterations();
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

bool StompPlanner::getSeedParameters(Eigen::MatrixXd& parameters) const
{
  using namespace utils::kinematics;
  using namespace utils::polynomial;

  auto within_tolerance = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b, double tol) -> bool
  {
    double dist = (a - b).cwiseAbs().sum();
    return dist <= tol;
  };

  trajectory_msgs::JointTrajectory traj;
  if(!extractSeedTrajectory(request_,traj))
  {
    ROS_DEBUG("%s Found no seed trajectory",getName().c_str());
    return false;
  }

  if(!jointTrajectorytoParameters(traj,parameters))
  {
    ROS_ERROR("%s Failed to created seed parameters from joint trajectory",getName().c_str());
    return false;
  }

  if(parameters.cols()<= 2)
  {
    ROS_ERROR("%s Found less than 3 points in seed trajectory",getName().c_str());
    return false;
  }

  /* ********************************************************************************
   * Validating seed trajectory by ensuring that it does obey the
   * motion plan request constraints
   */
  if(not isCartesianSeed())
  {
    moveit::core::RobotState state (robot_model_);
    const auto* group = state.getJointModelGroup(group_);
    const auto& joint_names = group->getActiveJointModelNames();
    const auto& tool_link = group->getLinkModelNames().back();
    Eigen::VectorXd start, goal;

    // We check to see if the start state in the request and the seed state are 'close'
    if (moveit::core::robotStateMsgToRobotState(request_.start_state, state))
    {
      // copying start joint values
      start.resize(joint_names.size());
      for(auto j = 0u; j < joint_names.size(); j++)
      {
        start(j) = state.getVariablePosition(joint_names[j]);
      }
      state.enforceBounds(group);

      if(within_tolerance(parameters.leftCols(1),start,MAX_START_DISTANCE_THRESH))
      {
        parameters.leftCols(1) = start;
      }
      else
      {
        ROS_ERROR("%s Start State is in discrepancy with the seed trajectory",getName().c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("%s Failed to get start state joints",getName().c_str());
      return false;
    }

    // We now extract the goal and make sure that the seed's goal obeys the goal constraints
    bool found_goal = false;
    goal = parameters.rightCols(1); // initializing goal;
    for(auto& gc : request_.goal_constraints)
    {
      if(!gc.joint_constraints.empty())
      {
        // copying goal values into state
        for(auto j = 0u; j < gc.joint_constraints.size() ; j++)
        {
          auto jc = gc.joint_constraints[j];
          state.setVariablePosition(jc.joint_name,jc.position);
        }

        // copying values into goal array
        if(!state.satisfiesBounds(group))
        {
          ROS_ERROR("%s Requested Goal joint pose is out of bounds",getName().c_str());
          continue;
        }

        for(auto j = 0u; j < joint_names.size(); j++)
        {
          goal(j) = state.getVariablePosition(joint_names[j]);
        }

        found_goal = true;
        break;
      }

      if(!gc.position_constraints.empty() && !gc.orientation_constraints.empty()) // checking cartesian
      {
        const double eps = 1e-3;
        const double timeout = 0.05;
        const std::string urdf_param = "/robot_description";
        const std::string base_frame = gc.position_constraints.front().header.frame_id;
        const std::string end_eff_frame = gc.position_constraints.front().link_name;
        std::map<std::string, double> fixed_joints = getFixedJointsMap(urdf_param, base_frame, end_eff_frame, group, request_.start_state.joint_state);
        TRAC_IK::TRAC_IK tracik_solver(base_frame, end_eff_frame, urdf_param, timeout, eps, fixed_joints);
        if(ikFromCartesianConstraints(gc.position_constraints.front(), gc.orientation_constraints.front(),
                                      group, goal, tracik_solver))
        {
          goal = filter(group, goal);
          found_goal = true;
          break;
        }
      }
    }

    // forcing the goal into the seed trajectory
    if(found_goal)
    {
      if(within_tolerance(parameters.rightCols(1),goal,MAX_START_DISTANCE_THRESH))
      {
        parameters.rightCols(1) = goal;
      }
      else
      {
        ROS_ERROR("%s Goal in seed to far away from Goal requested",getName().c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("%s Goal in seed to far away from Goal requested",getName().c_str());
      return false;
    }
  }

  if(!applyPolynomialSmoothing(robot_model_,group_,parameters,5,1e-5))
  {
    return false;
  }

  if(publish_seed_trajectory_)
    publishTrajectory(traj, seed_trajectory_publisher_);

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
      auto point = traj.points[step];
      double val = point.positions[joint];
      mat(joint, step) = val;
    }
  }

  parameters = mat;
  return true;
}

bool StompPlanner::extractSeedJointTrajectory(const moveit_msgs::MotionPlanRequest& req, trajectory_msgs::JointTrajectory& seed) const
{
  const auto* joint_group = robot_model_->getJointModelGroup(group_);
  const auto& names = joint_group->getActiveJointModelNames();
  const auto dof = names.size();

  const auto& constraints = req.trajectory_constraints.constraints; // alias to keep names short
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

bool StompPlanner::extractSeedCartesianTrajectory(const moveit_msgs::MotionPlanRequest& req, trajectory_msgs::JointTrajectory& seed) const
{
  using namespace moveit::core;
  using namespace utils::kinematics;
  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_);
  const auto& constraints = req.trajectory_constraints.constraints; // alias to keep names short

  ROS_ASSERT(constraints.size() == 1);
  ROS_ASSERT(constraints.front().position_constraints.size() == constraints.front().orientation_constraints.size());
  ROS_ASSERT(constraints.front().position_constraints.size() > 0);

  int fail_count = 0;

  const double eps = 1e-3;
  const double timeout = 0.05;
  const std::string urdf_param = "/robot_description";
  const std::string base_frame = constraints[0].position_constraints[0].header.frame_id;
  const std::string end_eff_frame = constraints[0].position_constraints[0].link_name;

  std::map<std::string, double> fixed_joints = getFixedJointsMap(urdf_param, base_frame, end_eff_frame, joint_group, request_.start_state.joint_state);
  TRAC_IK::TRAC_IK tracik_solver(base_frame, end_eff_frame, urdf_param, timeout, eps, fixed_joints);

  Eigen::VectorXd start_state;
  robotStateToEigen(req.start_state.joint_state, robot_model_, group_, start_state);

  ROS_ERROR_STREAM("Position constraints: " << constraints[0].position_constraints.size());

  for (auto i = 0; i < constraints[0].position_constraints.size(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint joint_pt;
    joint_pt.positions.resize(joint_group->getActiveJointModelNames().size(), 0.0);

    Eigen::VectorXd joint_pos;
    if(ikFromCartesianConstraints(constraints[0].position_constraints[i],
                                  constraints[0].orientation_constraints[i],
                                  joint_group,
                                  joint_pos,
                                  tracik_solver,
                                  start_state))
    {
      auto joint_pos2 = filter(joint_group, joint_pos);
      for(auto j=0; j<joint_pos2.size(); ++j)
        joint_pt.positions[j] = joint_pos2(j);
      //ROS_ERROR_STREAM("Start state shape: " << shape(start_state));
      //ROS_ERROR_STREAM("IK solution shape: " << shape(joint_pos2));
      start_state = joint_pos; // passing the previous joint_pos as hint for the next one!
      seed.points.push_back(joint_pt);
    }
    else
    {
      fail_count++;
      // if IK fails on the seed we should die here...instead now only pruningp
      ROS_ERROR_STREAM("**** FAILED TO IK CARTESIAN SEED at step " << i << " ******");
    }
  }

  ROS_WARN_STREAM("Seed trajectory converted with a total of " << fail_count << "/" << seed.points.size() << " IK FAILURES");
  ROS_YELLOW_STREAM("IK solver is using " << base_frame << " for base frame and " << end_eff_frame << " for end effector frame");

  seed.joint_names = joint_group->getActiveJointModelNames();

  double fail_percent = fail_count * 100.0 /constraints[0].position_constraints.size();
  if(fail_percent > 20)
  {
    ROS_ERROR_STREAM("Too many failed IK calls when converting!");
    return false;
  }

  return true;
}

bool StompPlanner::extractSeedTrajectory(const moveit_msgs::MotionPlanRequest& req, trajectory_msgs::JointTrajectory& seed) const
{
  if (req.trajectory_constraints.constraints.empty())
    return false;

  if(isCartesianSeed())
  {
    return extractSeedCartesianTrajectory(req, seed);
  }
  else
  {
    return extractSeedJointTrajectory(req, seed);
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

bool StompPlanner::isCartesianSeed() const
{
  const auto& constraints = request_.trajectory_constraints.constraints; // alias to keep names short
  ROS_ERROR_STREAM("We have a total of " << constraints.size() << " constraints");
  if (constraints[0].joint_constraints.size() > 0)
  {
    ROS_ERROR_STREAM("We have " << constraints[0].joint_constraints.size() << " joint constraints");
    return false;
  }

  if (constraints[0].position_constraints.size() > 0 and constraints[0].orientation_constraints.size() > 0)
  {
    ROS_ERROR_STREAM("We have " << constraints[0].position_constraints.size() << "  POSITION constraints");
    ROS_ERROR_STREAM("We have " << constraints[0].orientation_constraints.size() << "  ORIENTATION constraints");
    return true;
  }
}



bool StompPlanner::getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
  using namespace moveit::core;
  using namespace utils::kinematics;

  RobotStatePtr state(new RobotState(robot_model_));
  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_);
  std::string tool_link = joint_group->getLinkModelNames().back();
  bool found_goal = false;

  try
  {
    if(!robotStateToEigen(request_.start_state.joint_state, robot_model_, group_, start))
    {
      ROS_ERROR("%s Failed to extract start state from MotionPlanRequest",getName().c_str());
      ROS_ERROR_STREAM("************* " << request_.start_state.joint_state.name.size());
      return false;
    }

    const auto joint_names = state->getJointModelGroup(group_)->getActiveJointModelNames();
    goal.resize(joint_names.size());
    // check goal constraint
    if(request_.goal_constraints.empty())
    {
      ROS_ERROR("%s A goal constraint was not provided",getName().c_str());
      return false;
    }

    // extracting goal joint values
    for(const auto& gc : request_.goal_constraints)
    {
      if(!gc.joint_constraints.empty())
      {
        // copying goal values into state
        for(auto j = 0u; j < gc.joint_constraints.size() ; j++)
        {
          auto jc = gc.joint_constraints[j];
          state->setVariablePosition(jc.joint_name,jc.position);
        }

        if(!state->satisfiesBounds())
        {
          ROS_ERROR("%s Requested Goal joint pose is out of bounds",getName().c_str());
          continue;
        }

        ROS_DEBUG("%s Found goal from joint constraints",getName().c_str());

        // copying values into goal array
        for(auto j = 0u; j < joint_names.size(); j++)
        {
          goal(j) = state->getVariablePosition(joint_names[j]);
        }

        found_goal = true;
        break;

      }

      if(!gc.position_constraints.empty() && !gc.orientation_constraints.empty()) // check cartesian
      {
        // solving ik at goal
        const double eps = 1e-3;
        const double timeout = 0.05;
        const std::string urdf_param = "/robot_description";
        const std::string base_frame = gc.position_constraints.front().header.frame_id;
        const std::string end_eff_frame = gc.position_constraints.front().link_name;
        std::map<std::string, double> fixed_joints = getFixedJointsMap(urdf_param, base_frame, end_eff_frame, joint_group, request_.start_state.joint_state);
        TRAC_IK::TRAC_IK tracik_solver(base_frame, end_eff_frame, urdf_param, timeout, eps, fixed_joints);
        if(ikFromCartesianConstraints(gc.position_constraints.front(), gc.orientation_constraints.front(),
                                      joint_group, goal, tracik_solver))
        {
          goal = filter(joint_group, goal);
          found_goal = true;
          break;
        }
      }
      ROS_ERROR_COND(!found_goal,"%s was unable to retrieve the goal from the MotionPlanRequest",getName().c_str());
    }
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

  // make sure we have some constraints defined
  if (req.goal_constraints[0].joint_constraints.size() == 0 and
      req.goal_constraints[0].position_constraints.size() == 0 and
      req.goal_constraints[0].orientation_constraints.size() == 0)
  {
    ROS_ERROR("STOMP: No constraints defined!");
    return false;
  }

  // make sure we only have either joint constraints or cartesian constraints
  if (req.goal_constraints[0].joint_constraints.size() > 0 and
      req.goal_constraints[0].position_constraints.size() > 0 and
      req.goal_constraints[0].orientation_constraints.size() > 0)
  {
    ROS_ERROR("STOMP: Too many constraints defined! Can only accept either joint or Cartesian constraints!");
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

