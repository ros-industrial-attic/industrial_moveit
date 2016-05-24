#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <string.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <constrained_ik/moveit_interface/joint_interpolation_planner.h>
#include <constrained_ik/ConstrainedIKPlannerDynamicReconfigureConfig.h>
#include <fstream>
#include <time.h>
#include <pluginlib/class_loader.h>

using namespace ros;
using namespace constrained_ik;
using namespace Eigen;
using namespace moveit::core;
using namespace std;

typedef boost::shared_ptr<collision_detection::CollisionPlugin> CollisionPluginPtr;

int main (int argc, char *argv[])
{
  ros::init(argc,argv,"stomp_valgrid");
  ros::NodeHandle pnh;


  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
     ros::console::notifyLoggerLevelsChanged();

  sleep(3);
  robot_model_loader::RobotModelLoaderPtr loader;
  robot_model::RobotModelPtr robot_model;
  string urdf_file_path, srdf_file_path;

  urdf_file_path = package::getPath("stomp_test_support") + "/urdf/test_kr210l150_500K.urdf";
  srdf_file_path = package::getPath("stomp_test_kr210_moveit_config") + "/config/test_kr210.srdf";

  ifstream ifs1 (urdf_file_path.c_str());
  string urdf_string((istreambuf_iterator<char>(ifs1)), (istreambuf_iterator<char>()));

  ifstream ifs2 (srdf_file_path.c_str());
  string srdf_string((istreambuf_iterator<char>(ifs2)), (istreambuf_iterator<char>()));

  robot_model_loader::RobotModelLoader::Options opts(urdf_string, srdf_string);
  loader.reset(new robot_model_loader::RobotModelLoader(opts));
  robot_model = loader->getModel();

  if (!robot_model)
  {
    ROS_ERROR_STREAM("Unable to load robot model from urdf and srdf.");
    return false;
  }

//  boost::shared_ptr<pluginlib::ClassLoader<collision_detection::CollisionPlugin> > cd_loader(new pluginlib::ClassLoader<collision_detection::CollisionPlugin>("moveit_core", "collision_detection::CollisionPlugin"));
  collision_detection::CollisionPluginLoader cd_loader;
  ConstrainedIKPlannerDynamicReconfigureConfig config;
  config.joint_discretization_step = 0.02;
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  //Now assign collision detection plugin
  std::string class_name = "collision_detection/IndustrialMoveitCollisionDetection";
  cd_loader.setupScene(pnh, planning_scene);
  cd_loader.activate(class_name, planning_scene, true);
//  CollisionPluginPtr plugin = boost::shared_ptr<collision_detection::CollisionPlugin>(cd_loader->createUnmanagedInstance(class_name));
//  plugin->initialize(planning_scene, true);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  string group_name = "manipulator_rail";
  JointInterpolationPlanner joint_interp("", group_name);
  joint_interp.setConfiguration(config);

  req.allowed_planning_time = 10;
  req.num_planning_attempts = 1;
  req.group_name = group_name;

  robot_state::RobotState start = planning_scene->getCurrentState();
  map<string, double> jstart;
  jstart.insert(make_pair("joint_1", 1.4149));
  jstart.insert(make_pair("joint_2", 0.5530));
  jstart.insert(make_pair("joint_3", 0.1098));
  jstart.insert(make_pair("joint_4", -1.0295));
  jstart.insert(make_pair("joint_5", 0.0000));
  jstart.insert(make_pair("joint_6", 0.0000));
  jstart.insert(make_pair("rail_to_base", 1.3933));

  start.setVariablePositions(jstart);
  robotStateToRobotStateMsg(start, req.start_state);
  req.start_state.is_diff = true;

  robot_state::RobotState goal = planning_scene->getCurrentState();
  map<string, double> jgoal;
  jgoal.insert(make_pair("joint_1", 1.3060));
  jgoal.insert(make_pair("joint_2", -0.2627));
  jgoal.insert(make_pair("joint_3", 0.2985));
  jgoal.insert(make_pair("joint_4", -0.8236));
  jgoal.insert(make_pair("joint_5", 0.0000));
  jgoal.insert(make_pair("joint_6", 0.0000));
  jgoal.insert(make_pair("rail_to_base", -1.2584));

  goal.setVariablePositions(jgoal);

  vector<double> dist(7);
  dist[0] = 0.05;
  dist[1] = 0.05;
  dist[2] = 0.05;
  dist[3] = 0.05;
  dist[4] = 0.05;
  dist[5] = 0.05;
  dist[6] = 0.05;

  ros::Time t1, t2;
  t1 = ros::Time::now();
  const robot_state::JointModelGroup *jmg = goal.getJointModelGroup(group_name);
  int test_runs = 100;
  for (int i = 0; i < test_runs; i++)
  {
    if (jmg)
    {
      robot_state::RobotState new_goal = goal;
      new_goal.setToRandomPositionsNearBy(jmg, goal, dist);
      req.goal_constraints.resize(1);
      req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(new_goal, jmg);
    }

    joint_interp.clear();
    joint_interp.setPlanningScene(planning_scene);
    joint_interp.setMotionPlanRequest(req);

    if (!joint_interp.solve(res))
      ROS_ERROR("Joint Interpolated Solver failed (%d): %d", i, res.error_code_);

  }
  t2 = ros::Time::now();

  ROS_ERROR("Average time spent calculating trajectory: %4.10f seconds", (t2-t1).toSec()/test_runs);
  return 0;
}
