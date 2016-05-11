#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <string.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <stomp_moveit/stomp_planner.h>
#include <fstream>

using namespace ros;
using namespace stomp_moveit;
using namespace Eigen;
using namespace moveit::core;

int main (int argc, char *argv[])
{
  std::cout << "TEST" << std::endl;
  ros::init(argc,argv,"stomp_valgrid");
  ros::NodeHandle pnh;
  sleep(3);
  std::map<std::string, XmlRpc::XmlRpcValue> config;
  robot_model_loader::RobotModelLoaderPtr loader;
  robot_model::RobotModelPtr robot_model;
  bool active;
  std::string urdf_file_path, srdf_file_path;

  urdf_file_path = package::getPath("stomp_test_support") + "/urdf/test_kr210l150.urdf";
  srdf_file_path = package::getPath("stomp_test_kr210_moveit_config") + "/config/test_kr210.srdf";

  std::ifstream ifs1 (urdf_file_path.c_str());
  std::string urdf_string((std::istreambuf_iterator<char>(ifs1)), (std::istreambuf_iterator<char>()));

  std::ifstream ifs2 (srdf_file_path.c_str());
  std::string srdf_string((std::istreambuf_iterator<char>(ifs2)), (std::istreambuf_iterator<char>()));

  robot_model_loader::RobotModelLoader::Options opts(urdf_string, srdf_string);
  loader.reset(new robot_model_loader::RobotModelLoader(opts));
  robot_model = loader->getModel();

  if (!robot_model)
  {
    ROS_ERROR_STREAM("Could not load URDF model from " << urdf_file_path);
    ROS_ERROR_STREAM("Could not load SRDF model from " << srdf_file_path);
    active = false;
    return false;
  }

  StompPlanner::getConfigData(pnh, config);
  planning_scene::PlanningSceneConstPtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  std::string group_name = "manipulator_rail";
  StompPlanner stomp(group_name, config[group_name], robot_model);

  req.allowed_planning_time = 10;
  req.num_planning_attempts = 1;
  req.group_name = group_name;

  robot_state::RobotState start = planning_scene->getCurrentState();
  std::map<std::string, double> jstart;
  jstart.insert(std::make_pair("joint_1", 1.4149));
  jstart.insert(std::make_pair("joint_2", 0.5530));
  jstart.insert(std::make_pair("joint_3", 0.1098));
  jstart.insert(std::make_pair("joint_4", -1.0295));
  jstart.insert(std::make_pair("joint_5", 0.0000));
  jstart.insert(std::make_pair("joint_6", 0.0000));
  jstart.insert(std::make_pair("rail_to_base", 1.3933));

  start.setVariablePositions(jstart);
  robotStateToRobotStateMsg(start, req.start_state);
  req.start_state.is_diff = true;

  robot_state::RobotState goal = planning_scene->getCurrentState();
  std::map<std::string, double> jgoal;
  jgoal.insert(std::make_pair("joint_1", 1.3060));
  jgoal.insert(std::make_pair("joint_2", -0.2627));
  jgoal.insert(std::make_pair("joint_3", 0.2985));
  jgoal.insert(std::make_pair("joint_4", -0.8236));
  jgoal.insert(std::make_pair("joint_5", 0.0000));
  jgoal.insert(std::make_pair("joint_6", 0.0000));
  jgoal.insert(std::make_pair("rail_to_base", -1.2584));

  goal.setVariablePositions(jgoal);

  const robot_state::JointModelGroup *jmg = goal.getJointModelGroup(group_name);
  if (jmg)
  {
//    goal.setToRandomPositions(jmg);
    req.goal_constraints.resize(1);
    req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(goal, jmg);
  }

  stomp.clear();
  stomp.setPlanningScene(planning_scene);
  stomp.setMotionPlanRequest(req);

  if (!stomp.solve(res))
    ROS_ERROR_STREAM("STOMP Solver failed:" << res.error_code_);
  else
  {
    ROS_INFO("Finished");
    std::cout << "STD OUPUT" << std::endl;
  }

  return 0;
}
