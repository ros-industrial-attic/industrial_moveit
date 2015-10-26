
#include <constrained_ik/collision_robot_fcl_detailed.h>
#include <ros/ros.h>
#include <ctime>

namespace constrained_ik
{
  using namespace collision_detection;


  void CollisionRobotFCLDetailed::distanceSelf(const DistanceRequest &req, DistanceResult &res, const robot_state::RobotState &state) const
  {
    CollisionRobotFCLDetailed::distanceSelfDetailedHelper(req, res, state);
  }

  void CollisionRobotFCLDetailed::distanceSelfDetailedHelper(const DistanceRequest &req, DistanceResult &res, const robot_state::RobotState &state) const
  {
    FCLManager manager;
    CollisionRobotFCLDetailed::allocSelfCollisionBroadPhase(state, manager);
    DistanceData drd(&req, &res);

    manager.manager_->distance(&drd, &distanceDetailedCallback);
  }

}//namespace constrained_ik

