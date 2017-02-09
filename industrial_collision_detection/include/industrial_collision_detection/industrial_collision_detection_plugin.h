/**
 * @file industrial_collision_detection_plugin.h
 * @brief The industrial FCL plugin loader
 *
 * @author Levi Armstrong
 * @date May 4, 2016
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
#ifndef INDUSTRIAL_COLLISION_DETECTION_PLUGIN_H_
#define INDUSTRIAL_COLLISION_DETECTION_PLUGIN_H_
#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <industrial_collision_detection/collision_detection/collision_robot_industrial.h>
#include <industrial_collision_detection/collision_detection/collision_world_industrial.h>


namespace collision_detection
{
  /** @brief An allocator for Industrial Moveit FCL collision detectors */
  class CollisionDetectorAllocatorIndustrial :
      public collision_detection::CollisionDetectorAllocatorTemplate< CollisionWorldIndustrial,
      CollisionRobotIndustrial, CollisionDetectorAllocatorIndustrial >
  {
  public:
    static const std::string NAME_; // defined in collision_world_industrial.cpp
  };

  /**
   * @brief The Industrial FCL based Collision Plugin
   */
  class IndustrialFCLPluginLoader : public CollisionPlugin
  {
  public:
    virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const;
  };
}
#endif // INDUSTRIAL_COLLISION_DETECTION_PLUGIN_H_
