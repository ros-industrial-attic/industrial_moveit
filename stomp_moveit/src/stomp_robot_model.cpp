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
#include <XmlRpc.h>
#include <ros/package.h>



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

boost::shared_ptr<StompRobotModel>
loadStompRobotModel(ros::NodeHandle &nh, moveit::core::RobotModelConstPtr model)
{
  const std::string STOMP_ROBOT_PARAM = "stomp_robot";

  // load stomp robot model parameters
  XmlRpc::XmlRpcValue stomp_robot_param;
  boost::shared_ptr<stomp_moveit::StompRobotModel> stomp_robot_model;

  if (!nh.getParam(STOMP_ROBOT_PARAM, stomp_robot_param))
  {
    ROS_INFO("optional '%s' parameter not found, defaulting to moveit::core::RobotModel",STOMP_ROBOT_PARAM.c_str());
    stomp_robot_model.reset(new stomp_moveit::StompRobotModel(model));
    return stomp_robot_model;
  }

  if (!stomp_robot_param.hasMember("distance_field"))
  {
    ROS_ERROR("PlanningJobMoveitAdapter unable to parse ROS parameter:\n %s",STOMP_ROBOT_PARAM.c_str());
    throw std::logic_error("PlanningJobMoveitAdapter failure");
  }

  XmlRpc::XmlRpcValue distance_field;
  distance_field = stomp_robot_param["distance_field"];

  // No we must determine if we have a saved distance field or need to generate a new one
  // We do this by checking for the presence of the 'package' and 'relative_path' parameters

  if (distance_field.hasMember("package"))
  {
    ROS_INFO("Loading STOMP robot model from existing file");
    std::string package, relative_path;
    package = static_cast<std::string>(distance_field["package"]);
    relative_path = static_cast<std::string>(distance_field["relative_path"]);

    ROS_INFO("VDB file package: %s\nrelative_path: %s", package.c_str(), relative_path.c_str());
    const auto full_path = ros::package::getPath(package) + relative_path;
    stomp_robot_model.reset(new stomp_moveit::StompRobotModel(model, full_path));
  }
  else
  {
    ROS_INFO("Generating STOMP robot model from moveit::RobotModel");
    auto members = {"voxel_size","background"};
    for(auto& m : members)
    {
      if(!distance_field.hasMember(m))
      {
        ROS_ERROR("PlanningJobMoveitAdapter failed to find the '%s/%s/%s' parameter",STOMP_ROBOT_PARAM.c_str(),"distance_field",m);
        throw std::logic_error("PlanningJobMoveitAdapter failure");
      }
    }

    double df_voxel = static_cast<double>(distance_field["voxel_size"]);
    double df_background = static_cast<double>(distance_field["background"]);
    stomp_robot_model.reset(new stomp_moveit::StompRobotModel(model,df_voxel,df_background));
  }

  return stomp_robot_model;
}

} /* namespace stomp_moveit */
