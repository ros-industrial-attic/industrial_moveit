
#include <constrained_ik/collision_robot_fcl_detailed.h>
#include <ros/ros.h>
#include <ctime>

namespace constrained_ik
{
  using namespace collision_detection;


  CollisionRobotFCLDetailed::DistanceDetailedMap CollisionRobotFCLDetailed::distanceSelfDetailed(const robot_state::RobotState &state) const
  {
    return CollisionRobotFCLDetailed::distanceSelfDetailedHelper(state, NULL, NULL);
  }

  CollisionRobotFCLDetailed::DistanceDetailedMap CollisionRobotFCLDetailed::distanceSelfDetailed(const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
  {
    return CollisionRobotFCLDetailed::distanceSelfDetailedHelper(state, &acm, NULL);
  }

  CollisionRobotFCLDetailed::DistanceDetailedMap CollisionRobotFCLDetailed::distanceSelfDetailed(const robot_state::RobotState &state, const std::set<const robot_model::LinkModel*> &active_components_only) const
  {
    return CollisionRobotFCLDetailed::distanceSelfDetailedHelper(state, NULL, &active_components_only);
  }

  CollisionRobotFCLDetailed::DistanceDetailedMap CollisionRobotFCLDetailed::distanceSelfDetailed(const robot_state::RobotState &state, const AllowedCollisionMatrix &acm, const std::set<const robot_model::LinkModel*> &active_components_only) const
  {
    return CollisionRobotFCLDetailed::distanceSelfDetailedHelper(state, &acm, &active_components_only);
  }

  CollisionRobotFCLDetailed::DistanceDetailedMap CollisionRobotFCLDetailed::distanceSelfDetailedHelper(const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, const std::set<const robot_model::LinkModel*> *active_components_only) const
  {
    FCLManager manager;
    CollisionRobotFCLDetailed::allocSelfCollisionBroadPhase(state, manager);
    DistanceResultDetailed drd(acm, active_components_only);

    manager.manager_->distance(&drd, &CollisionRobotFCLDetailed::distanceDetailedCallback);

    return drd.distance_detailed_;
  }

  bool CollisionRobotFCLDetailed::distanceDetailedCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data, double& min_dist)
  {
    DistanceResultDetailed* cdata = reinterpret_cast<DistanceResultDetailed*>(data);

    const CollisionGeometryData* cd1 = static_cast<const CollisionGeometryData*>(o1->collisionGeometry()->getUserData());
    const CollisionGeometryData* cd2 = static_cast<const CollisionGeometryData*>(o2->collisionGeometry()->getUserData());
    const robot_model::LinkModel *l1, *l2;
    bool active1 = true, active2 = true;

    // If active components are specified
    if (cdata->active_components_only_)
    {
      l1 = cd1->type == BodyTypes::ROBOT_LINK ? cd1->ptr.link : (cd1->type == BodyTypes::ROBOT_ATTACHED ? cd1->ptr.ab->getAttachedLink() : NULL);
      l2 = cd2->type == BodyTypes::ROBOT_LINK ? cd2->ptr.link : (cd2->type == BodyTypes::ROBOT_ATTACHED ? cd2->ptr.ab->getAttachedLink() : NULL);

      // If neither of the involved components is active
      if ((!l1 || cdata->active_components_only_->find(l1) == cdata->active_components_only_->end()) &&
          (!l2 || cdata->active_components_only_->find(l2) == cdata->active_components_only_->end()))
      {
        return false;
      }
      else
      {
        if (!l1 || cdata->active_components_only_->find(l1) == cdata->active_components_only_->end())
          active1 = false;

        if (!l2 || cdata->active_components_only_->find(l2) == cdata->active_components_only_->end())
          active2 = false;
      }
    }

    // use the collision matrix (if any) to avoid certain distance checks
    bool always_allow_collision = false;
    if (cdata->acm_)
    {
      AllowedCollision::Type type;

      bool found = cdata->acm_->getAllowedCollision(cd1->getID(), cd2->getID(), type);
      if (found)
      {
        // if we have an entry in the collision matrix, we read it
        if (type == AllowedCollision::ALWAYS)
        {
          always_allow_collision = true;
          if (!cdata->verbose)
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
        if (!cdata->verbose)
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
          if (!cdata->verbose)
            logDebug("Robot link '%s' is allowed to touch attached object '%s'. No contacts are computed.",
                     cd2->getID().c_str(), cd1->getID().c_str());
        }
      }
    }

    if(always_allow_collision)
    {
      return false;
    }

    if (!cdata->verbose)
      logDebug("Actually checking collisions between %s and %s", cd1->getID().c_str(), cd2->getID().c_str());


    fcl::DistanceResult dist_result;
    double dist_threshold = std::numeric_limits<double>::max();
    std::map<std::string, fcl::DistanceResult>::iterator it1, it2;

    it1 = cdata->distance_detailed_.find(cd1->ptr.obj->id_);
    it2 = cdata->distance_detailed_.find(cd2->ptr.obj->id_);
    if (cdata->active_components_only_)
    {
      if (active1 && active2)
      {
        if (it1 != cdata->distance_detailed_.end() && it2 != cdata->distance_detailed_.end())
          dist_threshold = std::max(it1->second.min_distance, it2->second.min_distance);
      }
      else if (active1 && !active2)
      {
        if (it1 != cdata->distance_detailed_.end())
          dist_threshold = it1->second.min_distance;
      }
      else if (!active1 && active2)
      {
        if (it2 != cdata->distance_detailed_.end())
          dist_threshold = it2->second.min_distance;
      }
    }
    else
    {
      if (it1 != cdata->distance_detailed_.end() && it2 != cdata->distance_detailed_.end())
        dist_threshold = std::max(it1->second.min_distance, it2->second.min_distance);
    }

    dist_result.min_distance = dist_threshold;
    double d = fcl::distance(o1, o2, fcl::DistanceRequest(true), dist_result);

    // Check if either object is already in the map. If not add it or if present
    // check to see if the new distance is closer. If closer remove the existing
    // one and add the new distance information.
    if (d < dist_threshold)
    {
      if (it1 == cdata->distance_detailed_.end())
      {
        cdata->distance_detailed_.insert(std::make_pair<std::string, fcl::DistanceResult>(cd1->ptr.obj->id_, dist_result));
      }
      else
      {
        if (dist_result.min_distance < it1->second.min_distance)
        {
          cdata->distance_detailed_.erase(cd1->ptr.obj->id_);
          cdata->distance_detailed_.insert(std::make_pair<std::string, fcl::DistanceResult>(cd1->ptr.obj->id_, dist_result));
        }
      }

      if (it2 == cdata->distance_detailed_.end())
      {
        cdata->distance_detailed_.insert(std::make_pair<std::string, fcl::DistanceResult>(cd2->ptr.obj->id_, dist_result));
      }
      else
      {
        if (dist_result.min_distance < it2->second.min_distance)
        {
          cdata->distance_detailed_.erase(cd2->ptr.obj->id_);
          cdata->distance_detailed_.insert(std::make_pair<std::string, fcl::DistanceResult>(cd2->ptr.obj->id_, dist_result));
        }
      }
    }
    return false;
  }

  bool CollisionRobotFCLDetailed::getDistanceInfo(const DistanceDetailedMap &distance_detailed, DistanceInfoMap &distance_info_map)
  {
    bool status = true;
    for (DistanceDetailedMap::const_iterator it = distance_detailed.begin(); it != distance_detailed.end(); ++it)
    {
      DistanceInfo dist_info;
      fcl::DistanceResult dist = static_cast<const fcl::DistanceResult>(it->second);
      const collision_detection::CollisionGeometryData* cd1 = static_cast<const collision_detection::CollisionGeometryData*>(dist.o1->getUserData());
      const collision_detection::CollisionGeometryData* cd2 = static_cast<const collision_detection::CollisionGeometryData*>(dist.o2->getUserData());
      if (cd1->ptr.link->getName() == it->first)
      {
        dist_info.nearest_obsticle = cd2->ptr.link->getName();
        dist_info.link_point = Eigen::Vector3d(dist.nearest_points[0].data.vs);
        dist_info.obsticle_point = Eigen::Vector3d(dist.nearest_points[1].data.vs);
        dist_info.avoidance_vector = Eigen::Vector3d((dist.nearest_points[1]-dist.nearest_points[0]).data.vs);
        dist_info.avoidance_vector.norm();
        dist_info.distance = dist.min_distance;
      }
      else if (cd2->ptr.link->getName() == it->first)
      {
        dist_info.nearest_obsticle = cd1->ptr.link->getName();
        dist_info.link_point = Eigen::Vector3d(dist.nearest_points[1].data.vs);
        dist_info.obsticle_point = Eigen::Vector3d(dist.nearest_points[0].data.vs);
        dist_info.avoidance_vector = Eigen::Vector3d((dist.nearest_points[0]-dist.nearest_points[1]).data.vs);
        dist_info.avoidance_vector.norm();
        dist_info.distance = dist.min_distance;
      }
      else
      {
        ROS_ERROR("getDistanceInfo was unable to find link after match!");
        status &= false;
      }
      distance_info_map.insert(std::make_pair<std::string, DistanceInfo>(it->first, dist_info));
    }

    return status;
  }

  void CollisionRobotFCLDetailed::DistanceResultDetailed::enableGroup(const robot_model::RobotModelConstPtr &kmodel)
  {
    if (kmodel->hasJointModelGroup(group_name))
      active_components_only_ = &kmodel->getJointModelGroup(group_name)->getUpdatedLinkModelsWithGeometrySet();
    else
      active_components_only_ = NULL;
  }
}//namespace constrained_ik

