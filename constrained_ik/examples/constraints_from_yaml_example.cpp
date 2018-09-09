#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/enum_types.h"
#include "example_utils.h"
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace constrained_ik;
using namespace constrained_ik::basic_kin;
using namespace Eigen;

/**
 * @brief This is an example shows how to add constraint/constraints
 * using a yaml file loaded to the ros param server.
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "constraints_from_yaml_example");
  Constrained_IK ik;
  BasicKin kin;
  robot_model_loader::RobotModelLoaderPtr loader;
  planning_scene::PlanningScenePtr planning_scene;

  // Load example robot model
  planning_scene = getExampleRobotData(loader);

  // Initialize kinematic model
  kin.init(loader->getModel()->getJointModelGroup("manipulator"));

  // Add constraint/constraints from ros parameter server
  std::string param = "example_yaml/constraints";
  ik.addConstraintsFromParamServer(param);

  // Initialize Solver
  ik.init(kin);

  // Calculate IK for sample case
  evaluteExampleIK(kin, ik, planning_scene);

  return 0;
}
