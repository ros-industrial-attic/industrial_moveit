/**
 * @file collision_common.h
 * @brief This contains common data used during collision checking
 *
 * This add additional capability not found in the existing MoveIt
 * implementation. The add capability is the ability to make detailed
 * distance requests.
 *
 * @author Levi Armstrong
 * @date May 4, 2016
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
#ifndef COLLISION_DETECTION_COLLISION_COMMON_H_
#define COLLISION_DETECTION_COLLISION_COMMON_H_

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <set>

namespace collision_detection
{
  bool distanceDetailedCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data, double& min_dist);
}

#endif
