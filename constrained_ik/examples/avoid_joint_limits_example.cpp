#include "constrained_ik/constraints/avoid_joint_limits.h"
#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/enum_types.h"
#include "example_utils.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace constrained_ik;
using namespace constrained_ik::basic_kin;
using namespace Eigen;

/** @brief This is an example showing how to add a AvoidJointLimits constraint
 * using c++ */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "avoid_joint_limits_example");
  Constrained_IK ik;
  BasicKin kin;
  robot_model_loader::RobotModelLoaderPtr loader;

  // Load example robot model
  getExampleRobotData(loader);

  // Initialize kinematic model
  kin.init(loader->getModel()->getJointModelGroup("manipulator"));

  // Create constraint
  constraints::AvoidJointLimits *constraint = new constraints::AvoidJointLimits;
  constraint->setDebug(false);
  constraint->setWeight(1.0);
  constraint->setThreshold(0.05);

  // Add constraint to solver
  ik.addConstraint(constraint, constraint_types::Primary);

  // Initialize Solver
  ik.init(kin);

  // Calculate IK for sample case
  evaluteExampleIK(kin, ik);

  return 0;
}
