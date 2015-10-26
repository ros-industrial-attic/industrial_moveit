#include <constrained_ik/collision_world_fcl_detailed.h>
#include <constrained_ik/collision_robot_fcl_detailed.h>

namespace constrained_ik
{
  using namespace collision_detection;

  void CollisionWorldFCLDetailed::distanceRobot(const DistanceRequest &req, DistanceResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state) const
  {
    CollisionWorldFCLDetailed::distanceRobotDetailedHelper(req, res, robot, state);
  }

  void CollisionWorldFCLDetailed::distanceRobotDetailedHelper(const DistanceRequest &req, DistanceResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state) const
  {
    const CollisionRobotFCLDetailed& robot_fcl = dynamic_cast<const CollisionRobotFCLDetailed&>(robot);
    FCLObject fcl_obj;
    robot_fcl.constructFCLObject(state, fcl_obj);

    DistanceData drd(&req, &res);
    for(std::size_t i = 0; !drd.done && i < fcl_obj.collision_objects_.size(); ++i)
      manager_->distance(fcl_obj.collision_objects_[i].get(), &drd, &distanceDetailedCallback);

  }

}
