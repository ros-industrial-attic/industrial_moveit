/**
 * @file obstacle_distance_gradient.cpp
 * @brief This defines a Robot Model for the Stomp Planner.
 *
 * @author Jorge Nicho
 * @date Jul 22, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
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

#include <stomp_moveit/cost_functions/obstacle_distance_gradient.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::ObstacleDistanceGradient,stomp_moveit::cost_functions::StompCostFunction)


namespace stomp_moveit
{
namespace cost_functions
{

ObstacleDistanceGradient::ObstacleDistanceGradient() :
    name_("ObstacleDistanceGradient"),
    robot_state_()
{

}

ObstacleDistanceGradient::~ObstacleDistanceGradient()
{

}

bool ObstacleDistanceGradient::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                                          const std::string& group_name, XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  collision_request_.distance = true;
  collision_request_.group_name = group_name;
  collision_request_.cost = false;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = false;
  collision_request_.verbose = false;
  return configure(config);
}

bool ObstacleDistanceGradient::configure(const XmlRpc::XmlRpcValue& config)
{

  try
  {
    // check parameter presence
    auto members = {"cost_weight" ,"max_distance"};
    for(auto& m : members)
    {
      if(!config.hasMember(m))
      {
        ROS_ERROR("%s failed to find the '%s' parameter",getName().c_str(),m);
        return false;
      }
    }

    XmlRpc::XmlRpcValue c = config;
    max_distance_ = static_cast<double>(c["max_distance"]);
    cost_weight_ = static_cast<double>(c["cost_weight"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

bool ObstacleDistanceGradient::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    const moveit_msgs::MotionPlanRequest &req,
                                                    const stomp_core::StompConfiguration &config,
                                                    moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  planning_scene_ = planning_scene;
  plan_request_ = req;
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // storing robot state
  robot_state_.reset(new RobotState(robot_model_ptr_));
  if(!robotStateMsgToRobotState(req.start_state,*robot_state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  return true;
}

bool ObstacleDistanceGradient::computeCosts(const Eigen::MatrixXd& parameters, std::size_t start_timestep,
                                            std::size_t num_timesteps, int iteration_number, int rollout_number,
                                            Eigen::VectorXd& costs, bool& validity)
{

  if(!robot_state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }

  // allocating
  costs = Eigen::VectorXd::Zero(num_timesteps);
  const moveit::core::JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);

  if(parameters.cols()<start_timestep + num_timesteps)
  {
    ROS_ERROR_STREAM("Size in the 'parameters' matrix is less than required");
    return false;
  }

  // request the distance at each state
  double cost;
  double dist;
  validity = true;
  for (auto t=start_timestep; t<start_timestep + num_timesteps; ++t)
  {
    collision_result_.clear();
    robot_state_->setJointGroupPositions(joint_group,parameters.col(t));
    robot_state_->update();
    collision_result_.distance = max_distance_;

    planning_scene_->checkSelfCollision(collision_request_,collision_result_,*robot_state_,planning_scene_->getAllowedCollisionMatrix());
    dist = collision_result_.collision ? -1.0 :collision_result_.distance ;

    if(dist >= max_distance_)
    {
      cost = 0; // away from obstacle
    }
    else if(dist < 0)
    {
      cost = 1.0; // in collision
      validity = false;
    }
    else
    {
      cost = (max_distance_ - dist)/max_distance_;
    }

    costs(t) = cost;
  }

  return true;
}

void ObstacleDistanceGradient::done(bool success,int total_iterations,double final_cost)
{
  robot_state_.reset();
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
