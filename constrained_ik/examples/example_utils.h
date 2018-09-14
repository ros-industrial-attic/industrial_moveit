#ifndef EXAMPLE_UTILS_H
#define EXAMPLE_UTILS_H
#include "constrained_ik/basic_kin.h"
#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constrained_ik_utils.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <ros/package.h>
#include <fstream>

/**
 * @brief Get the example Robot Model Ptr
 * @return RobotModelPtr
 */
static planning_scene::PlanningScenePtr getExampleRobotData(robot_model_loader::RobotModelLoaderPtr &loader)
{
  ros::NodeHandle nh;
  std::string urdf_file_path, srdf_file_path;

  urdf_file_path = ros::package::getPath("constrained_ik") + "/test/resources/ur10.urdf";
  srdf_file_path = ros::package::getPath("constrained_ik") + "/test/resources/ur10.srdf";

  std::ifstream ifs1 (urdf_file_path.c_str());
  std::string urdf_string((std::istreambuf_iterator<char>(ifs1)), (std::istreambuf_iterator<char>()));

  std::ifstream ifs2 (srdf_file_path.c_str());
  std::string srdf_string((std::istreambuf_iterator<char>(ifs2)), (std::istreambuf_iterator<char>()));

  robot_model_loader::RobotModelLoader::Options opts(urdf_string, srdf_string);
  loader.reset(new robot_model_loader::RobotModelLoader(opts));

  // Create planning scene and assign collision detection plugin
  collision_detection::CollisionPluginLoader cd_loader;
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(loader->getModel()));

  std::string class_name = "FCL";
  cd_loader.setupScene(nh, planning_scene);
  cd_loader.activate(class_name, planning_scene, true);
  return planning_scene;
}

/**
 * @brief Evaluate the provide ik solver
 * @param kin kinematic model
 * @param ik inverse kinematic solver
 * @param planning_scene collision environment model,
 * Not every constraint requires the planning scene.
 * @return true if successfull, otherwize false
 */
static bool evaluteExampleIK(constrained_ik::basic_kin::BasicKin &kin, constrained_ik::Constrained_IK &ik, planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScenePtr())
{
  ros::NodeHandle nh;
  Eigen::Affine3d goalPose, finalPose;
  Eigen::VectorXd joints(6);

  // get data from ros param server
  std::vector<double> goal_vector, seed_vector;
  nh.getParam("/constrained_ik/example/goal", goal_vector);
  nh.getParam("/constrained_ik/example/seed", seed_vector);
  Eigen::VectorXd goal = Eigen::VectorXd::Map(goal_vector.data(), goal_vector.size());
  Eigen::VectorXd seed = Eigen::VectorXd::Map(seed_vector.data(), seed_vector.size());

  constrained_ik::ConstrainedIKConfiguration config;
  config = ik.getSolverConfiguration();
  config.solver_min_iterations = 5;
  ik.setSolverConfiguration(config);

  kin.calcFwdKin(goal, goalPose);

  // Now use the ik solver to see if it can find a joint solution for
  // homePose give seed.
  if (ik.calcInvKin(goalPose, seed, planning_scene, joints))
  {
    Eigen::IOFormat vector_fmt(4, 0, ", ", "\n", "[", "]");
    ROS_INFO("IK Solution Found");
    ROS_INFO_STREAM("Desired Joint State: " << goal.transpose().format(vector_fmt));
    ROS_INFO_STREAM("Seed Joint State:    " << seed.transpose().format(vector_fmt));
    ROS_INFO_STREAM("Final Joint State:   " << joints.transpose().format(vector_fmt));

    Eigen::IOFormat matrix_fmt(4, 0, ", ", "\n", "                                [", "]");
    kin.calcFwdKin(joints, finalPose);
    ROS_INFO_STREAM("IK Desired Pose:\n" << goalPose.matrix().format(matrix_fmt));
    ROS_INFO_STREAM("IK Final Pose:\n" << finalPose.matrix().format(matrix_fmt));
    return true;
  }

  ROS_INFO("Unable to find IK Solution");
  return false;
}

#endif // EXAMPLE_UTILS_H
