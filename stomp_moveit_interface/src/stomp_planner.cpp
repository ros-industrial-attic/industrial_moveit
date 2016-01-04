/*
 * stomp_planner.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <stomp_moveit_interface/stomp_planner.h>
#include <stomp/stomp_utils.h>
#include <class_loader/class_loader.h>

namespace stomp_moveit_interface
{

const static double DEFAULT_CONTROL_COST_WEIGHT = 0.1; //0.0001;
const static int TERMINATION_ATTEMPTS = 200;
const double TERMINATION_DELAY = 0.1f;

StompPlanner::StompPlanner(const std::string& group,const moveit::core::RobotModelConstPtr& model):
    PlanningContext("STOMP",group),
    node_handle_("~"),
    solving_(false),
    stomp_()
{
  trajectory_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("stomp_trajectory", 20);
  init(model);

}

StompPlanner::~StompPlanner()
{
}

void StompPlanner::init(const moveit::core::RobotModelConstPtr& model)
{
  kinematic_model_ = model;
  if(!getPlanningScene())
  {
    setPlanningScene(planning_scene::PlanningSceneConstPtr(new planning_scene::PlanningScene(model)));
  }

  // loading parameters
  int max_rollouts;
  XmlRpc::XmlRpcValue features_xml;

  STOMP_VERIFY(node_handle_.getParam("features", features_xml));
  STOMP_VERIFY(node_handle_.getParam("max_rollouts", max_rollouts));


  // Stomp Optimization Task setup
  stomp_task_.reset(new StompOptimizationTask(request_.group_name,
                                              getPlanningScene()));

  if(stomp_task_->initialize(max_rollouts) &&
      stomp_task_->setFeaturesFromXml(features_xml))
  {
    stomp_task_->setControlCostWeight(DEFAULT_CONTROL_COST_WEIGHT);
    stomp_task_->setTrajectoryVizPublisher(const_cast<ros::Publisher&>(trajectory_viz_pub_));
  }
  else
  {
    ROS_ERROR_STREAM("Stomp Optimization Task failed to initialize");
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
  // initializing response
  res.description_.resize(1);
  res.description_[0] = getDescription();
  res.processing_time_.resize(1);
  res.trajectory_.resize(1);
  ros::WallTime start_time = ros::WallTime::now();
  bool success = false;
  stomp_.reset(new stomp::STOMP());
  setSolving(true);

  if(!stomp_task_->setMotionPlanRequest(planning_scene_, request_, res.error_code_))
  {
    ROS_ERROR("STOMP failed to set MotionPlanRequest ");
    return false;
  }

  if(!stomp_->initialize(node_handle_, stomp_task_))
  {
    ROS_ERROR_STREAM("STOMP Optimizer initialization failed");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    setSolving(false);
    return success;
  }

  ROS_DEBUG_STREAM("STOMP planning started");
  success = getSolving() && stomp_->runUntilValid();

  if(success)
  {
    std::vector<Eigen::VectorXd> best_params;
    double best_cost;
    stomp_->getBestNoiselessParameters(best_params, best_cost);
    trajectory_msgs::JointTrajectory trajectory;

    if(stomp_task_->parametersToJointTrajectory(best_params, trajectory))
    {
      stomp_task_->publishResultsMarkers(best_params);

      moveit::core::RobotState robot_state(planning_scene_->getRobotModel());
      moveit::core::robotStateMsgToRobotState(request_.start_state,robot_state);
      res.trajectory_.resize(1);
      res.trajectory_[0]= robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(
          kinematic_model_,request_.group_name));
      res.trajectory_.back()->setRobotTrajectoryMsg( robot_state,trajectory);

      if(planning_scene_ && !planning_scene_->isPathValid(*res.trajectory_.back(),group_,true))
      {
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        success = false;
        ROS_ERROR_STREAM("STOMP generated an invalid path");
      }
      else
      {
        ros::WallDuration wd = ros::WallTime::now() - start_time;
        res.processing_time_[0] = ros::Duration(wd.sec, wd.nsec).toSec();
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        ROS_INFO_STREAM("STOMP found a valid path after "<<res.processing_time_[0]<<" seconds");
      }
    }
    else
    {

      res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      success = false;
      ROS_ERROR_STREAM("STOMP returned an invalid motion plan");
    }
  }
  else
  {

    ROS_ERROR("STOMP failed to find a collision-free plan");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
  }

  setSolving(false);
  return success;
}

bool StompPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  // check if planner is available
  if (req.planner_id != "STOMP" && req.planner_id != "CHOMP")
  {
    ROS_ERROR("STOMP: Planner %s not available.", req.planner_id.c_str());
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

bool StompPlanner::terminate()
{
  int num_attempts = TERMINATION_ATTEMPTS;
  bool success = false;
  ros::Duration delay(TERMINATION_DELAY);
  if(getSolving())
  {
    setSolving(false);
    if(stomp_)
    {
      stomp_->proceed(false);
    }

    ROS_WARN("STOMP planner terminated");
  }
  else
  {
    success = true;
    ROS_WARN("STOMP planner terminated");
  }


  return success;
}

// thread save methods
bool StompPlanner::getSolving()
{
  solving_mutex_.lock();
  bool r = solving_;
  solving_mutex_.unlock();
  return r;
}

void StompPlanner::setSolving(bool solve)
{
  solving_mutex_.lock();
  solving_ = solve;
  solving_mutex_.unlock();
}

void StompPlanner::clear()
{
  stomp_.reset();
  setSolving(false);

}


} /* namespace stomp_moveit_interface */

