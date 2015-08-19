/*
 * stomp_planner.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#include <ros/ros.h>
#include <stomp_moveit_interface/stomp_optimization_task.h>
#include <stomp_moveit_interface/stomp_planner.h>
#include <stomp/stomp_utils.h>
#include <class_loader/class_loader.h>

namespace stomp_moveit_interface
{

const static int DEFAULT_MAX_ITERATIONS = 100;
const static int DEFAULT_ITERATIONS_AFTER_COLLISION_FREE = 20 ;
const static double DEFAULT_CONTROL_COST_WEIGHT = 0.001;
const static double DEFAULT_SCALE = 1.0;
const static double DEFAULT_PADDING = 0.05f;
const static int OPTIMIZATION_TASK_THREADS = 1;
const static bool USE_SIGNED_DISTANCE_FIELD = true;
const static int TERMINATION_ATTEMPTS = 200;
const double TERMINATION_DELAY = 0.1f;

StompPlanner::StompPlanner(const std::string& group,const moveit::core::RobotModelConstPtr& model):
    PlanningContext("STOMP",group),
    node_handle_("~"),
    solving_(false),
    stomp_()
{
  trajectory_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("stomp_trajectory", 20);
  robot_body_viz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("stomp_robot", 20);
  init(model);

}

StompPlanner::~StompPlanner()
{
}

void StompPlanner::init(const moveit::core::RobotModelConstPtr& model)
{
  kinematic_model_ = model;

  // read distance field params
  double sx, sy ,sz, orig_x,orig_y,orig_z;
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_x", sx));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_y", sy));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_z", sz));
  STOMP_VERIFY(node_handle_.getParam("collision_space/origin_x", orig_x));
  STOMP_VERIFY(node_handle_.getParam("collision_space/origin_y", orig_y));
  STOMP_VERIFY(node_handle_.getParam("collision_space/origin_z", orig_z));
  STOMP_VERIFY(node_handle_.getParam("collision_space/resolution", df_resolution_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/collision_tolerance", df_collision_tolerance_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/max_propagation_distance", df_max_propagation_distance_));

  df_size_ = Eigen::Vector3d(sx,sy,sz);
  df_origin_ = Eigen::Vector3d(orig_x,orig_y,orig_z);

  // initializing collision components
  if(planning_scene_)
  {
    last_planning_scene_ = planning_scene_->diff();
  }
  else
  {
    last_planning_scene_.reset(new planning_scene::PlanningScene(model));
  }

  // creating collision world representation
  ros::Time start_time =ros::Time::now();
  ROS_INFO_STREAM("Collision World Distance Field creation started");
  collision_world_df_.reset(new collision_detection::CollisionWorldDistanceField(df_size_,
                                                                              df_origin_,
                                                                              USE_SIGNED_DISTANCE_FIELD,
                                                                              df_resolution_, df_collision_tolerance_,
                                                                              df_max_propagation_distance_));
  copyObjects(last_planning_scene_->getCollisionWorld(), collision_world_df_);
  ROS_INFO_STREAM("Collision World Distance Field completed in "<< (ros::Time::now() - start_time).toSec()<<" seconds");


  // creating collision robot representation
  start_time = ros::Time::now();
  ROS_INFO_STREAM("Collision Robot Distance Field creation started");
  collision_robot_df_.reset(
      new collision_detection::CollisionRobotDistanceField(*last_planning_scene_->getCollisionRobot(),
                                                          df_size_,
                                                          df_origin_,
                                                          USE_SIGNED_DISTANCE_FIELD,
                                                          df_resolution_,
                                                          df_collision_tolerance_,
                                                          df_max_propagation_distance_,
                                                          DEFAULT_PADDING));
  ROS_INFO_STREAM("Collision Robot Distance Field completed in "<< (ros::Time::now() - start_time).toSec()<<" seconds");
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

void StompPlanner::updateCollisionModels(planning_scene::PlanningSceneConstPtr& current_planning_scene)
{
  bool reset_collision_robot = false;
  bool reset_collision_world = false;
  planning_scene::PlanningScenePtr scene_diff;

  // checking for changes in collision world
  if(!last_planning_scene_)
  {
    ROS_DEBUG_STREAM("Last planning scene is empty, creating new one with the current planning scene as a parent");
    last_planning_scene_ = current_planning_scene->diff();
    reset_collision_world = true;
  }
  else
  {
    if((!last_planning_scene_->getParent()) ||
        ( last_planning_scene_->getParent()->getName() != current_planning_scene->getName() ))
    {
      ROS_DEBUG_STREAM("Parent scene in last saved scene is invalid, creating new one with the current planning scene as a parent");
      last_planning_scene_ = current_planning_scene->diff();
      scene_diff = planning_scene::PlanningScene::clone(current_planning_scene);
      reset_collision_world = true;
    }
    else
    {
      moveit_msgs::PlanningScene scene_diff_msg;
      scene_diff = planning_scene::PlanningScene::clone(last_planning_scene_);
      last_planning_scene_->getPlanningSceneDiffMsg(scene_diff_msg);
      scene_diff->setPlanningSceneMsg(scene_diff_msg);
      reset_collision_world = !scene_diff->getWorld()->getObjectIds().empty();
    }
  }

  std::stringstream ss;
  scene_diff->printKnownObjects(ss);
  ROS_DEBUG_STREAM("Planning Scene Diff Objects\n"<<ss.str());



  // check if objects were attached to the robot
  std::vector<const robot_state::AttachedBody*> attached_bodies;
  scene_diff->getCurrentState().getAttachedBodies(attached_bodies);
  for(unsigned int i = 0 ; i < attached_bodies.size(); i++)
  {
    if(!last_planning_scene_->getCurrentState().hasAttachedBody(attached_bodies[i]->getName()))
    {
      reset_collision_robot = true;
      break;
    }
  }

  if(reset_collision_world)
  {
    ros::Time start_time = ros::Time::now();
    ROS_INFO_STREAM("Collision World Distance Field creation started");
    collision_world_df_.reset(new collision_detection::CollisionWorldDistanceField(df_size_,
                                                                                df_origin_,
                                                                                USE_SIGNED_DISTANCE_FIELD,
                                                                                df_resolution_, df_collision_tolerance_,
                                                                                df_max_propagation_distance_));
    copyObjects(current_planning_scene->getCollisionWorld(), collision_world_df_);

    ROS_INFO_STREAM("Collision World Distance Field creation completed after "
        <<(ros::Time::now() - start_time).toSec()<<" seconds");
  }

  if(reset_collision_robot)
  {
    ros::Time start_time = ros::Time::now();
    ROS_INFO_STREAM("Collision Robot Distance Field creation started");
    collision_robot_df_.reset(
        new collision_detection::CollisionRobotDistanceField(*current_planning_scene->getCollisionRobot(),
                                                            df_size_,
                                                            df_origin_,
                                                            USE_SIGNED_DISTANCE_FIELD,
                                                            df_resolution_,
                                                            df_collision_tolerance_,
                                                            df_max_propagation_distance_,
                                                            DEFAULT_PADDING));
    ROS_INFO_STREAM("Collision Robot Distance Field creation completed after "
        <<(ros::Time::now() - start_time).toSec()<<" seconds");

  }

  // saving last scene
  if(last_planning_scene_ != current_planning_scene->diff())
  {
    last_planning_scene_ = current_planning_scene->diff();
  }

}

bool StompPlanner::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  setSolving(true);
  stomp_.reset(new stomp::STOMP());
  ros::WallTime start_time = ros::WallTime::now();
  boost::shared_ptr<StompOptimizationTask> stomp_task;


  // prepare the collision checkers
  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot = planning_scene_->getCollisionRobot();
  boost::shared_ptr<const collision_detection::CollisionWorld> collision_world = planning_scene_->getCollisionWorld();

  // check for termination
  if(!stomp_->getProceed())
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    res.trajectory_.resize(1);
    setSolving(false);
    return false;
  }

  if(planning_scene_)
  {
    updateCollisionModels(planning_scene_);
  }
  else
  {
    ROS_DEBUG_STREAM("Planning scene not set skipping update of collision models");
  }

  // check for termination
  if(!stomp_->getProceed())
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    res.trajectory_.resize(1);
    setSolving(false);
    return false;
  }

  // optimization task setup
  int max_rollouts;
  int num_threads=OPTIMIZATION_TASK_THREADS;
  STOMP_VERIFY(node_handle_.getParam("max_rollouts", max_rollouts));
  stomp_task.reset(new StompOptimizationTask(node_handle_, request_.group_name,
                                             kinematic_model_,
                                             collision_robot, collision_world,
                                             collision_robot_df_, collision_world_df_));
  STOMP_VERIFY(stomp_task->initialize(num_threads, max_rollouts));
  XmlRpc::XmlRpcValue features_xml;
  STOMP_VERIFY(node_handle_.getParam("features", features_xml));
  stomp_task->setFeaturesFromXml(features_xml);
  stomp_task->setControlCostWeight(DEFAULT_CONTROL_COST_WEIGHT);
  stomp_task->setTrajectoryVizPublisher(const_cast<ros::Publisher&>(trajectory_viz_pub_));
  stomp_task->setCollisionRobotMarkerPublisher(robot_body_viz_pub_);
  stomp_task->setDistanceFieldMarkerPublisher(trajectory_viz_pub_);
  stomp_task->setMotionPlanRequest(planning_scene_, request_);


  // check for termination
  if(!stomp_->getProceed())
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    res.trajectory_.resize(1);
    setSolving(false);
    return false;
  }

  ros::Time start = ros::Time::now();
  stomp_->initialize(node_handle_, stomp_task);
  ROS_DEBUG_STREAM("STOMP planning started");
  bool success = stomp_->runUntilValid(DEFAULT_MAX_ITERATIONS,DEFAULT_ITERATIONS_AFTER_COLLISION_FREE);
  ROS_DEBUG_STREAM("STOMP planning " <<(success ? "completed" : "failed"));

  std::vector<Eigen::VectorXd> best_params;
  double best_cost;
  stomp_->getBestNoiselessParameters(best_params, best_cost);
  stomp_task->publishResultsMarkers(best_params);
  trajectory_msgs::JointTrajectory trajectory;

  res.description_.resize(1);
  res.description_[0] = getDescription();
  res.processing_time_.resize(1);
  res.trajectory_.resize(1);
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.processing_time_[0] = ros::Duration(wd.sec, wd.nsec).toSec();

  if(success)
  {
    if(stomp_task->parametersToJointTrajectory(best_params, trajectory))
    {

      moveit::core::RobotState robot_state(planning_scene_->getRobotModel());
      res.trajectory_.resize(1);
      res.trajectory_[0]= robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(
          kinematic_model_,request_.group_name));
      res.trajectory_.back()->setRobotTrajectoryMsg( robot_state,trajectory);
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      ROS_INFO_STREAM("STOMP found a motion plan after "<<res.processing_time_[0]<<" seconds");
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

  if(stomp_)
  {
    stomp_->proceed(false);
    for(unsigned int i = 0 ; i < num_attempts ; i++)
    {
      delay.sleep();
      if(!getSolving())
      {
        success = true;
        ROS_WARN("STOMP planner terminated");
        break;
      }
    }
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



void StompPlanner::copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                 const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const
{
  std::vector<std::string> object_ids = from_world->getWorld()->getObjectIds();
  for (size_t i=0; i<object_ids.size(); ++i)
  {
    collision_detection::CollisionWorld::ObjectConstPtr obj = from_world->getWorld()->getObject(object_ids[i]);
    to_world->getWorld()->addToObject(object_ids[i], obj->shapes_, obj->shape_poses_);
  }
}

} /* namespace stomp_moveit_interface */

