#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/enum_types.h"
#include "constrained_ik/constraints/avoid_obstacles.h"
#include "example_utils.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace constrained_ik;
using namespace constrained_ik::basic_kin;
using namespace Eigen;

/** @brief This is an example showing how to add a AvoidObstacles constraint using c++ */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "avoid_obstacles_example");
  Constrained_IK ik;
  BasicKin kin;
  robot_model_loader::RobotModelLoaderPtr loader;
  planning_scene::PlanningScenePtr planning_scene;

  // Load example robot model
  planning_scene = getExampleRobotData(loader);

  // Initialize kinematic model
  kin.init(loader->getModel()->getJointModelGroup("manipulator"));

  // Create constraint
  std::vector<std::string> link_names = boost::assign::list_of("upper_arm_link")("wrist_3_link");
  constraints::AvoidObstacles *constraint = new constraints::AvoidObstacles;
  constraint->setAvoidanceLinks(link_names);
  constraint->setAmplitude(link_names[0], 0.01);
  constraint->setAmplitude(link_names[1], 0.01);
  constraint->setMinDistance(link_names[0], 0.01);
  constraint->setMinDistance(link_names[1], 0.01);
  constraint->setAvoidanceDistance(link_names[0], 1.0);
  constraint->setAvoidanceDistance(link_names[1], 1.0);
  constraint->setWeight(link_names[0], 1.0);
  constraint->setWeight(link_names[1], 1.0);
  constraint->setDebug(false);

  // Add constraint to solver
  ik.addConstraint(constraint, constraint_types::Primary);

  // Initialize Solver
  ik.init(kin);

  // Calculate IK for sample case
  evaluteExampleIK(kin, ik, planning_scene);

  return 0;
}
