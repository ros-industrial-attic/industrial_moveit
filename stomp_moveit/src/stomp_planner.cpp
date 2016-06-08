/*
 * stomp_planner.cpp
 *
 *  Created on: April 4, 2016
 *      Author: Jorge Nicho
 */

#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <stomp_moveit/stomp_planner.h>
#include <class_loader/class_loader.h>
#include <stomp_core/utils.h>


const std::string DESCRIPTION = "STOMP";

namespace stomp_moveit
{

StompPlanner::StompPlanner(const std::string& group,const XmlRpc::XmlRpcValue& config,
                           const moveit::core::RobotModelConstPtr& model):
    PlanningContext(DESCRIPTION,group),
    config_(config),
    robot_model_(model),
    seed_provided_(false)
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
    std::string group;
    XmlRpc::XmlRpcValue task_config;
    task_config = config_["task"];
    task_.reset(new StompOptimizationTask(robot_model_,group_,task_config));

    // parsing stomp parameters
    if(!config_.hasMember("optimization") || !stomp_core::Stomp::parseConfig(config_["optimization" ],stomp_config_))
    {
      std::string msg = "Stomp 'optimization' parameter for group '" + group_ + "' was not found";
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

  if (seed_provided_)
  {
    auto config_copy = stomp_config_;
    config_copy.num_timesteps = seed_traj_.points.size();

    // setting up up optimization task
    if(!task_->setMotionPlanRequest(planning_scene_, request_, config_copy, res.error_code_))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    Eigen::MatrixXd initial_parameters;
    jointTrajectorytoParameters(seed_traj_, initial_parameters);

    planning_success = stomp_->solve(initial_parameters, parameters);
  }
  else
  {
    // setting up up optimization task
    if(!task_->setMotionPlanRequest(planning_scene_,request_, stomp_config_,res.error_code_))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    // extracting start and goal
    std::vector<double> start, goal;
    if(!getStartAndGoal(start,goal))
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      ROS_ERROR("Stomp failed to get the start and goal positions");
      return false;
    }

    planning_success = stomp_->solve(start,goal,parameters);
  }

  // invalidate seed
  seed_provided_ = false;

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
    ROS_ERROR_STREAM("Stomp Trajectory is in collision");
  }

  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.processing_time_[0] = ros::Duration(wd.sec, wd.nsec).toSec();
  ROS_INFO_STREAM("STOMP found a valid path after "<<res.processing_time_[0]<<" seconds");

  return true;
}

bool StompPlanner::parametersToJointTrajectory(Eigen::MatrixXd& parameters, trajectory_msgs::JointTrajectory& trajectory)
{

  // checking parameters dimensions
  if(parameters.rows() != stomp_config_.num_dimensions)
  {
    ROS_ERROR_STREAM("Parameters array dimensions "<<parameters.rows()<<"do not match the expected number of dimensions"
                     <<stomp_config_.num_dimensions<<" JointTrajectory message will not be created");
    return false;
  }

  if(parameters.cols() != stomp_config_.num_timesteps)
  {
    ROS_ERROR_STREAM("Parameters array time steps "<<parameters.cols()<<"do not match the expected number of points"
                     <<stomp_config_.num_timesteps<<" JointTrajectory message will not be created");
    return false;
  }

  // computing velocities and accelerations
  Eigen::MatrixXd vels(parameters), accs(parameters);
  vels.setZero();
  accs.setZero();
  Eigen::VectorXd array;
  for (auto d=0u; d < parameters.rows(); ++d)
  {
    stomp_core::differentiate(parameters.row(d),
                              stomp_core::DerivativeOrders::STOMP_VELOCITY,  stomp_config_.delta_t,array);
    vels.row(d) = array;
    stomp_core::differentiate(parameters.row(d),
                              stomp_core::DerivativeOrders::STOMP_ACCELERATION, stomp_config_.delta_t,array);
    accs.row(d) = array;
  }

  // filling trajectory joint values
  trajectory.joint_names = robot_model_->getJointModelGroup(group_)->getActiveJointModelNames();
  trajectory.points.clear();
  trajectory.points.resize(parameters.cols());
  std::vector<double> vals(parameters.rows());
  for(auto t = 0u; t < parameters.cols() ; t++)
  {
    Eigen::VectorXd::Map(&vals[0],vals.size()) = parameters.col(t);
    trajectory.points[t].positions = vals;

    Eigen::VectorXd::Map(&vals[0],vals.size()) = vels.col(t);
    trajectory.points[t].velocities = vals;

    Eigen::VectorXd::Map(&vals[0],vals.size()) = accs.col(t);
    trajectory.points[t].accelerations = vals;

    trajectory.points[t].time_from_start = ros::Duration(t*stomp_config_.delta_t);
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

bool StompPlanner::getStartAndGoal(std::vector<double>& start, std::vector<double>& goal)
{
  using namespace moveit::core;
  RobotStatePtr state(new RobotState(robot_model_));
  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_);

  try
  {
    // copying start state
    if(!robotStateMsgToRobotState(request_.start_state,*state))
    {
      ROS_ERROR_STREAM("Failed to extract start state from MotionPlanRequest");
      return false;
    }

    // copying start joint values
    const std::vector<std::string> joint_names= state->getJointModelGroup(group_)->getActiveJointModelNames();
    start.resize(joint_names.size());
    goal.resize(joint_names.size());
    state->enforceBounds(joint_group);
    for(auto j = 0u; j < joint_names.size(); j++)
    {
      start[j] = state->getVariablePosition(joint_names[j]);
    }

    // extracting goal joint values
    for(auto& gc: request_.goal_constraints)
    {
      if(gc.joint_constraints.empty())
      {
        ROS_ERROR_STREAM("No joint values for the goal were found");
        return false;
      }

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
        goal[j] = state->getVariablePosition(joint_names[j]);
      }

      break;
    }

  }
  catch(moveit::Exception &e)
  {
    ROS_ERROR("Failure retrieving start or goal state joint values from request %s", e.what());
    return false;
  }

  return true;

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
  if (req.goal_constraints[0].position_constraints.size() > 0
      || req.goal_constraints[0].orientation_constraints.size() > 0
      || req.goal_constraints[0].visibility_constraints.size() > 0
      || req.goal_constraints[0].joint_constraints.size() == 0)
  {
    ROS_ERROR("STOMP: Can only handle joint space goals.");
    return false;
  }

  return true;
}

bool StompPlanner::setInitialTrajectory(const trajectory_msgs::JointTrajectory& initial_traj)
{
  auto dof = initial_traj.joint_names.size();
  if (dof != stomp_config_.num_dimensions)
  {
    ROS_WARN("STOMP Unable to set seed trajectory: DOF in planner must equal STOMP configuration (%d vs %d)",
             static_cast<int>(dof), stomp_config_.num_dimensions);
    return false;
  }

  if (initial_traj.points.size() < 2)
  {
    ROS_WARN("STOMP unable to set seed trajectory: Seed trajectory must have at least 2 points");
    return false;
  }

  if (initial_traj.points.back().time_from_start.toSec() == 0.0)
  {
    ROS_WARN("STOMP unable to set seed trajectory: Seed trajectory must have non-zero ending point");
    return false;
  }

  // It passed, so record it into the object
  seed_provided_ = true;
  seed_traj_ = initial_traj;

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

