/**
 * @file stomp_robot_model.cpp
 * @brief This defines a Robot Model for the Stomp Planner.
 *
 * @author Jorge Nicho
 * @date Jul 28, 2016
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

#include <ros/console.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <stomp_moveit/stomp_robot_model.h>

namespace stomp_moveit
{

StompRobotModel::StompRobotModel(moveit::core::RobotModelConstPtr robot_model,double df_voxel,double df_background_distance):
  df_background_distance_(df_background_distance),
  df_voxel_size_(df_voxel),
  moveit::core::RobotModel(*robot_model),
  collision_robot_df_()
{
  // initialize distance field based Collision Robot
  if(!collision_robot_df_)
  {
    ros::Time start = ros::Time::now();
    ROS_INFO("StompRobotModel creating distance field");
    double bandwidth = df_background_distance_/df_voxel_size_;
    collision_robot_df_.reset(new distance_field::CollisionRobotOpenVDB(robot_model,df_voxel_size_,df_background_distance_,bandwidth, bandwidth));
    ros::Duration duration = ros::Time::now() - start;
    ROS_INFO("StompRobotModel completed distance field after %f seconds", duration.toSec());

  }

}

StompRobotModel::StompRobotModel(moveit::core::RobotModelConstPtr robot_model):
  df_background_distance_(0),
  df_voxel_size_(0),
  moveit::core::RobotModel(*robot_model),
  collision_robot_df_()
{
  ROS_WARN("StompRobotModel has been instantiated without a distance field");
}

StompRobotModel::StompRobotModel(moveit::core::RobotModelConstPtr robot_model, const std::string &saved_sdf)
  : moveit::core::RobotModel(*robot_model)
{
  collision_robot_df_.reset(new distance_field::CollisionRobotOpenVDB(robot_model, saved_sdf));
}

StompRobotModel::~StompRobotModel()
{
  // TODO Auto-generated destructor stub
}

double StompRobotModel::distance(const std::string& group,planning_scene::PlanningSceneConstPtr planning_scene,
                                 const robot_state::RobotState &state) const
{
  if(!collision_robot_df_)
  {
    ROS_ERROR("Distance field has not been initialized");
    return std::nan("l");
  }

  collision_detection::DistanceRequest distance_request;
  collision_detection::DistanceResult distance_result;

  distance_request.acm = &planning_scene->getAllowedCollisionMatrix();
  distance_request.group_name = group;

  collision_robot_df_->distanceSelf(distance_request,distance_result,state);
  return distance_result.minimum_distance.min_distance;
}

} /* namespace stomp_moveit */
