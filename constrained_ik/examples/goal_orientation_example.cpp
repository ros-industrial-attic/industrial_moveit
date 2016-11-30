#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/enum_types.h"
#include "constrained_ik/constraints/goal_orientation.h"
#include "example_utils.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace constrained_ik;
using namespace constrained_ik::basic_kin;
using namespace Eigen;

/** @brief This is an example showing how to add a GoalOrientation constraint using c++ */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "goal_orientation_example");
  Constrained_IK ik;
  BasicKin kin;
  robot_model_loader::RobotModelLoaderPtr loader;

  // Load example robot model
  getExampleRobotData(loader);

  // Initialize kinematic model
  kin.init(loader->getModel()->getJointModelGroup("manipulator"));

  // Create constraint
  constraints::GoalOrientation *constraint = new constraints::GoalOrientation;
  constraint->setDebug(false);
  constraint->setWeight(Eigen::Vector3d::Ones());
  constraint->setTolerance(0.009);

  // Add constraint to solver
  ik.addConstraint(constraint, constraint_types::Primary);

  // Initialize Solver
  ik.init(kin);

  // Calculate IK for sample case
  evaluteExampleIK(kin, ik);

  return 0;
}
