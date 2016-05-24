#ifndef INDUSTRIAL_COLLISION_DETECTION_PLUGIN_H
#define INDUSTRIAL_COLLISION_DETECTION_PLUGIN_H
#include <moveit/collision_detection/collision_plugin.h>
#include <industrial_collision_detection/collision_detector_allocator_industrial.h>
namespace collision_detection
{
  class IndustrialMoveitCollisionDetectionLoader : public CollisionPlugin
  {
  public:
    virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const;
  };
}
#endif // INDUSTRIAL_COLLISION_DETECTION_PLUGIN_H
