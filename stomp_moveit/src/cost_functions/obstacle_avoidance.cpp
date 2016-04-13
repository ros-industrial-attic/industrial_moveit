/*
 * obstacle_avoidance.cpp
 *
 *  Created on: Mar 30, 2016
 *      Author: Jorge Nicho
 */
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include "stomp_moveit/cost_functions/obstacle_avoidance.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::ObstacleAvoidance,stomp_moveit::cost_functions::StompCostFunction);

const double DEFAULT_PADDING = 0.0;
const double DEFAULT_SCALE = 1.0;

namespace stomp_moveit
{
namespace cost_functions
{

ObstacleAvoidance::ObstacleAvoidance():
    name_("ObstacleAvoidancePlugin")
{
  // TODO Auto-generated constructor stub

}

ObstacleAvoidance::~ObstacleAvoidance()
{
  // TODO Auto-generated destructor stub
}

bool ObstacleAvoidance::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  return configure(config);
}

bool ObstacleAvoidance::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 int num_timesteps,
                 double dt,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  planning_scene_ = planning_scene;
  plan_request_ = req;
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // initialize collision request
  collision_request_.group_name = group_name_;
  collision_request_.cost = false;
  collision_request_.distance = true;
  collision_request_.max_contacts = 0;
  collision_request_.max_contacts_per_pair = 0;
  collision_request_.contacts = false;
  collision_request_.verbose = false;

  collision_robot_.reset(new collision_detection::CollisionRobotFCLDetailed(
      planning_scene_->getRobotModel(), DEFAULT_PADDING, DEFAULT_SCALE, collision_clearance_));
  collision_world_.reset(new collision_detection::CollisionWorldFCLDetailed(
      boost::const_pointer_cast<collision_detection::World>(planning_scene_->getWorld()), collision_clearance_));

  return true;
}

bool ObstacleAvoidance::computeCosts(const Eigen::MatrixXd& parameters,
                          std::size_t start_timestep,
                          std::size_t num_timesteps,
                          int iteration_number,
                          int rollout_number,
                          Eigen::VectorXd& costs,
                          bool& validity) const
{

  using namespace moveit::core;

  typedef collision_detection::CollisionResult::ContactMap ContactMap;
  typedef ContactMap::iterator ContactMapIterator;
  typedef std::vector<collision_detection::Contact> ContactArray;

  // initializing result array
  costs = Eigen::VectorXd::Zero(num_timesteps);

  // collision
  collision_detection::CollisionRequest request = collision_request_;
  collision_detection::CollisionResult result_world_collision, result_robot_collision;
  std::vector<collision_detection::CollisionResult> results(2);
  validity = true;

  // robot state
  const JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);
  RobotStatePtr robot_state_ptr(new RobotState(robot_model_ptr_));
  std::vector<double> joint_values(parameters.rows(),0);

  if(parameters.cols()<start_timestep + num_timesteps)
  {
    ROS_ERROR_STREAM("Size in the 'parameters' matrix is less than required");
    return false;
  }

  // iterating through collisions
  for (auto t=start_timestep; t<start_timestep + num_timesteps; ++t)
  {
    robot_state_ptr->setJointGroupPositions(joint_group,parameters.col(t));
    robot_state_ptr->update();

    // checking robot vs world (attached objects, octomap, not in urdf) collisions
    result_world_collision.distance = std::numeric_limits<double>::max();

    collision_world_->checkRobotCollision(request,
                                          result_world_collision,
                                          *collision_robot_,
                                          *robot_state_ptr,
                                          planning_scene_->getAllowedCollisionMatrix());

    collision_robot_->checkSelfCollision(request,
                                         result_robot_collision,
                                         *robot_state_ptr,
                                         planning_scene_->getAllowedCollisionMatrix());

    results[0]= result_world_collision;
    results[1] = result_robot_collision;

    double penalty = 0;
    double distance = collision_clearance_;
    bool collision = false;
    for(std::vector<collision_detection::CollisionResult>::iterator i = results.begin(); i != results.end(); i++)
    {
      collision_detection::CollisionResult& result = *i;

      // get shortest distance
      distance = distance > result.distance ? result.distance : distance;
      collision |= result.collision;
    }

    if(collision)
    {
      costs(t) = collision_penalty_;
      validity = false;
    }
    else
    {
      costs(t) = distance > collision_clearance_  ? 0 : (collision_clearance_ - distance);
    }
  }

  // scaling cost
  double max = costs.maxCoeff();
  costs /= (max > 1e-8) ? max : 1;

  return true;
}

bool ObstacleAvoidance::configure(const XmlRpc::XmlRpcValue& config)
{
  try
  {
    XmlRpc::XmlRpcValue c = config;
    collision_clearance_ = static_cast<double>(c["collision_clearance"]);
    collision_penalty_ = static_cast<double>(c["collision_penalty"]);
    cost_weight_ = static_cast<double>(c["cost_weight"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
