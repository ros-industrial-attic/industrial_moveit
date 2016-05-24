#include <industrial_collision_detection/industrial_collision_detection_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace collision_detection
{
  bool IndustrialMoveitCollisionDetectionLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
  {
    scene->setActiveCollisionDetector(CollisionDetectorAllocatorIndustrial::create(), exclusive);
    return true;
  }
}

PLUGINLIB_EXPORT_CLASS(collision_detection::IndustrialMoveitCollisionDetectionLoader, collision_detection::CollisionPlugin)
