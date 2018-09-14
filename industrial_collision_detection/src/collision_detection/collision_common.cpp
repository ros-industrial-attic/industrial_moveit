/**
 * @file collision_common.cpp
 * @brief This contains common data used during collision checking
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
#include <industrial_collision_detection/collision_detection/collision_common.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <ros/ros.h>
#include <console_bridge/console.h>

namespace collision_detection
{
  bool distanceDetailedCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& min_dist)
  {
    DistanceData* cdata = reinterpret_cast<DistanceData*>(data);

    const CollisionGeometryData* cd1 = static_cast<const CollisionGeometryData*>(o1->collisionGeometry()->getUserData());
    const CollisionGeometryData* cd2 = static_cast<const CollisionGeometryData*>(o2->collisionGeometry()->getUserData());
    bool active1 = true, active2 = true;

    // do not distance check geoms part of the same object / link / attached body
    if (cd1->sameObject(*cd2))
      return false;

    // If active components are specified
    if (cdata->req->active_components_only)
    {
      const robot_model::LinkModel *l1 = cd1->type == BodyTypes::ROBOT_LINK ? cd1->ptr.link : (cd1->type == BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : NULL);
      const robot_model::LinkModel *l2 = cd2->type == BodyTypes::ROBOT_LINK ? cd2->ptr.link : (cd2->type == BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : NULL);

      // If neither of the involved components is active
      if ((!l1 || cdata->req->active_components_only->find(l1) == cdata->req->active_components_only->end()) &&
          (!l2 || cdata->req->active_components_only->find(l2) == cdata->req->active_components_only->end()))
      {
        return false;
      }
      else
      {
        if (!l1 || cdata->req->active_components_only->find(l1) == cdata->req->active_components_only->end())
          active1 = false;

        if (!l2 || cdata->req->active_components_only->find(l2) == cdata->req->active_components_only->end())
          active2 = false;
      }
    }

    // use the collision matrix (if any) to avoid certain distance checks
    bool always_allow_collision = false;
    if (cdata->req->acm)
    {
      AllowedCollision::Type type;

      bool found = cdata->req->acm->getAllowedCollision(cd1->getID(), cd2->getID(), type);
      if (found)
      {
        // if we have an entry in the collision matrix, we read it
        if (type == AllowedCollision::ALWAYS)
        {
          always_allow_collision = true;
          if (!cdata->req->verbose)
            logDebug("Collision between '%s' and '%s' is always allowed. No contacts are computed.",
                     cd1->getID().c_str(), cd2->getID().c_str());
        }
      }
    }

    // check if a link is touching an attached object
    if (cd1->type == BodyTypes::ROBOT_LINK && cd2->type == BodyTypes::ROBOT_ATTACHED)
    {
      const std::set<std::string> &tl = cd2->ptr.ab->getTouchLinks();
      if (tl.find(cd1->getID()) != tl.end())
      {
        always_allow_collision = true;
        if (!cdata->req->verbose)
          logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                   cd1->getID().c_str(), cd2->getID().c_str());
      }
    }
    else
    {
      if (cd2->type == BodyTypes::ROBOT_LINK && cd1->type == BodyTypes::ROBOT_ATTACHED)
      {
        const std::set<std::string> &tl = cd1->ptr.ab->getTouchLinks();
        if (tl.find(cd2->getID()) != tl.end())
        {
          always_allow_collision = true;
          if (!cdata->req->verbose)
            logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                     cd2->getID().c_str(), cd1->getID().c_str());
        }
      }
    }

    if(always_allow_collision)
    {
      return false;
    }

    if (!cdata->req->verbose)
      logDebug("Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());


    fcl::DistanceResult fcl_result;
    DistanceResultsData dist_result;
    double dist_threshold = cdata->req->distance_threshold;
    std::map<std::string, DistanceResultsData>::iterator it1, it2;

    /*
     * FIXME: The old Distance Query Data structures that existed in this package were replaced by their counterparts in the
     * MoveIt! library.  Unfortunately the legacy code here isn't fully compatible with the new structure types and  so for
     * now the broken code below has been disabled until the discrepancies get amended.
     */
    if (cdata->req->type == DistanceRequestType::GLOBAL)
    {
/*      it1 = cdata->res->distance.find(cd1->ptr.obj->id_);
      it2 = cdata->res->distance.find(cd2->ptr.obj->id_);

      if (cdata->req->active_components_only)
      {
        if (active1 && active2)
        {
          if (it1 != cdata->res->distance.end() && it2 != cdata->res->distance.end())
            dist_threshold = std::max(it1->second.min_distance, it2->second.min_distance);
        }
        else if (active1 && !active2)
        {
          if (it1 != cdata->res->distance.end())
            dist_threshold = it1->second.min_distance;
        }
        else if (!active1 && active2)
        {
          if (it2 != cdata->res->distance.end())
            dist_threshold = it2->second.min_distance;
        }
      }
      else
      {
        if (it1 != cdata->res->distance.end() && it2 != cdata->res->distance.end())
          dist_threshold = std::max(it1->second.min_distance, it2->second.min_distance);
      }*/
    }
    else
    {
        dist_threshold = cdata->res->minimum_distance.distance;
    }

    fcl_result.min_distance = dist_threshold;
    double d = fcl::distance(o1, o2, fcl::DistanceRequest{.enable_nearest_point = true}, fcl_result);

    // Check if either object is already in the map. If not add it or if present
    // check to see if the new distance is closer. If closer remove the existing
    // one and add the new distance information.
    if (d < dist_threshold)
    {
      dist_result.distance = fcl_result.min_distance;
      dist_result.nearest_points[0] = Eigen::Vector3d(fcl_result.nearest_points[0].data.vs);
      dist_result.nearest_points[1] = Eigen::Vector3d(fcl_result.nearest_points[1].data.vs);
      dist_result.link_names[0] = cd1->ptr.obj->id_;
      dist_result.link_names[1] = cd2->ptr.obj->id_;

      cdata->res->minimum_distance = dist_result;

      /*
        * FIXME: The old Distance Query Data structures that existed in this package were replaced by their counterparts in the
        * MoveIt! library.  Unfortunately the legacy code here isn't fully compatible with the new structure types and  so for
        * now the broken code below has been disabled until the discrepancies get amended.
        */
      if (cdata->req->type == DistanceRequestType::GLOBAL)
      {
        /*
        if (d <= 0 && !cdata->res->collision)
        {
          cdata->res->collision = true;
        }

        if (active1)
        {
          if (it1 == cdata->res->distances.end())
          {
            cdata->res->distances.insert(std::make_pair(cd1->ptr.obj->id_, dist_result));
          }
          else
          {
            it1->second.update(dist_result);
          }
        }

        if (active2)
        {
          if (it2 == cdata->res->distance.end())
          {
            cdata->res->distance.insert(std::make_pair(cd2->ptr.obj->id_, dist_result));
          }
          else
          {
            it2->second.update(dist_result);
          }
        }
        */
      }
      else
      {
        if (d <= 0)
        {
          cdata->res->collision = true;
          cdata->done = true;
        }
      }
    }

    return cdata->done;
  }
}
