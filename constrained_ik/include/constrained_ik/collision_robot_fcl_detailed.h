#ifndef COLLISION_ROBOT_FCL_DETAILED_H
#define COLLISION_ROBOT_FCL_DETAILED_H

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <eigen3/Eigen/Core>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace constrained_ik
{
  class CollisionRobotFCLDetailed : public collision_detection::CollisionRobotFCL
  {
  public:
    typedef boost::shared_ptr<CollisionRobotFCLDetailed> CollisionRobotFCLDetailedPtr;
    typedef std::map<std::string, fcl::DistanceResult> DistanceDetailedMap;

    struct DistanceResultDetailed
    {
      DistanceResultDetailed() : active_components_only_(NULL), acm_(NULL), verbose(false)
      {
      }

      DistanceResultDetailed(const collision_detection::AllowedCollisionMatrix *acm, const std::set<const robot_model::LinkModel*> *active_components_only) : active_components_only_(active_components_only), acm_(acm), verbose(false)
      {
      }

      ~DistanceResultDetailed()
      {
      }

      /// Compute \e active_components_only_ based on \e req_
      void enableGroup(const robot_model::RobotModelConstPtr &kmodel);

      /// If the collision request includes a group name, this set contains the pointers to the link models that are considered for collision;
      /// If the pointer is NULL, all collisions are considered.
      const std::set<const robot_model::LinkModel*> *active_components_only_;

      /// The user specified collision matrix (may be NULL)
      const collision_detection::AllowedCollisionMatrix *acm_;

      bool verbose;

      std::string group_name;

      DistanceDetailedMap distance_detailed_;

    };

    CollisionRobotFCLDetailed(const robot_model::RobotModelConstPtr &kmodel, double padding = 0.0, double scale = 1.0): CollisionRobotFCL(kmodel, padding, scale) {}

    CollisionRobotFCLDetailed(const CollisionRobotFCLDetailed &other): CollisionRobotFCL(other) {}

    virtual DistanceDetailedMap distanceSelfDetailed(const robot_state::RobotState &state) const;

    virtual DistanceDetailedMap distanceSelfDetailed(const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm) const;

    virtual DistanceDetailedMap distanceSelfDetailed(const robot_state::RobotState &state, const std::set<const robot_model::LinkModel*> &active_components_only) const;

    virtual DistanceDetailedMap distanceSelfDetailed(const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm, const std::set<const robot_model::LinkModel*> &active_components_only) const;

    virtual DistanceDetailedMap distanceSelfDetailedHelper(const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix *acm, const std::set<const robot_model::LinkModel *> *active_components_only) const;

    static bool distanceDetailedCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data, double& min_dist);

    /** @brief Containst distance information in the planning frame queried from getDistanceInfo() */
    struct DistanceInfo
    {
      std::string nearest_obsticle; /**< The link name for nearest obsticle/link to request link. */
      Eigen::Vector3d link_point; /**< Point on request link */
      Eigen::Vector3d obsticle_point; /**< Point on nearest link to requested link */
      Eigen::Vector3d avoidance_vector; /**< Normilized Vector created by nearest points */
      double distance; /**< Distance between nearest points */

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /**
     * @brief getDistanceInfo
     * @param link_name Requested link name for distance information
     * @param dist_info Stores the distance information for requested link
     * @return bool, true if distance information exists.
     */
    static bool getDistanceInfo(const DistanceDetailedMap distance_detailed, const std::string link_name, DistanceInfo & dist_info);

  };
} //namespace constrained_ik
#endif // COLLISION_ROBOT_FCL_DETAILED_H

