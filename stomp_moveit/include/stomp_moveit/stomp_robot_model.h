/**
 * @file stomp_robot_model.h
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

#ifndef INCLUDE_STOMP_MOVEIT_STOMP_ROBOT_MODEL_H_
#define INCLUDE_STOMP_MOVEIT_STOMP_ROBOT_MODEL_H_

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <openvdb_distance_field.h>

namespace stomp_moveit
{

class StompRobotModel : public moveit::core::RobotModel
{
public:
  StompRobotModel(moveit::core::RobotModelConstPtr robot_model,double df_voxel,double df_max_distance);
  StompRobotModel(moveit::core::RobotModelConstPtr robot_model);
  virtual ~StompRobotModel();

  /*
   * @brief Returns the minimum distance between a link in the robot group and an obstacle.
   */
  double distance(const std::string& group,planning_scene::PlanningSceneConstPtr planning_scene, const robot_state::RobotState &state) const;
  bool hasDistanceField() const
  {
    return static_cast<bool> (collision_robot_df_);
  }

protected:

  double df_background_distance_;
  double df_voxel_size_;
  boost::shared_ptr<distance_field::CollisionRobotOpenVDB> collision_robot_df_;
};

} /* namespace stomp_moveit */

#endif /* INCLUDE_STOMP_MOVEIT_STOMP_ROBOT_MODEL_H_ */
