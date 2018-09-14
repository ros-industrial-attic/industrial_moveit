/**
 * @file test_constrained_ik.cpp
 * @brief Test Fixtures
 *
 * Consolidate variable-definitions and init functions for use by multiple tests.
 *
 * @author dsolomon
 * @date Sep 23, 2013
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/goal_pose.h"
#include "constrained_ik/constraints/goal_position.h"
#include "constrained_ik/constraints/goal_orientation.h"
#include "constrained_ik/constraints/avoid_obstacles.h"
#include "constrained_ik/constrained_ik_utils.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <boost/random/uniform_int_distribution.hpp>

using constrained_ik::Constrained_IK;
using constrained_ik::basic_kin::BasicKin;
using Eigen::Affine3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::JacobiSVD;


const std::string GROUP_NAME = "manipulator"; /**< Default group name for tests */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

/** @brief This test the Constrained_IK null space project calculation */
TEST(constrained_ik, nullspaceprojection)
{
  int rows = 6;
  int cols = 8;
  MatrixXd A = MatrixXd::Random(rows, cols);
  
  JacobiSVD<MatrixXd> svd(A,Eigen::ComputeFullV | Eigen::ComputeFullU);
  MatrixXd V(svd.matrixV());
  MatrixXd U(svd.matrixU());
  Eigen::MatrixXd S(rows,cols);
  // TODO learn how to initialize eigen matrices with zero()
  for(int i=0; i<rows; i++)
  {
    for(int j=0; j<cols; j++) S(i,j) = 0.0;
  }
  VectorXd s = svd.singularValues();
  for(int i=0; i<rows-3; i++)
  {
    S(i,i) = s(i);
  }

  MatrixXd A2 = U * S * V.transpose();

  Constrained_IK CIK;
   
  MatrixXd P1 = CIK.calcNullspaceProjectionTheRightWay(A2);
  EXPECT_EQ(P1.rows(), cols);
  EXPECT_EQ(P1.cols(), cols);
  MatrixXd P2 = CIK.calcNullspaceProjection(A2);
  EXPECT_EQ(P2.rows(), cols);
  EXPECT_EQ(P2.cols(), cols);
  VectorXd testv1 = VectorXd::Random(cols);
  VectorXd P1v = P1*testv1;
  VectorXd P2v = P2*testv1;
  VectorXd AP1v = A2*P1v;
  VectorXd AP2v = A2*P2v;
  double norm1 = AP1v.norm();
  double norm2 = AP2v.norm();
  EXPECT_LT(norm1, .00000001);
  EXPECT_LT(norm2, .00000001);
}

/**
 * @brief Constrained_IK Test Fixtures
 * Consolidate variable-definitions and init functions for use by multiple tests.
 */
class BasicIKTest : public ::testing::Test
{
protected:

  robot_model_loader::RobotModelLoaderPtr loader_;  /**< Used to load the robot model */
  moveit::core::RobotModelPtr robot_model_; /**< Robot model */
  planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene for the current robot model */
  BasicKin kin; /**< Basic Kinematic Model of the robot.  */
  Constrained_IK ik; /**< The Constrained IK Solver */
  Affine3d homePose; /**< Cartesian home position */
  constrained_ik::ConstrainedIKConfiguration config; /**< Constrained IK configuration parameters */

  /** @brief See base class for documention */
  virtual void SetUp()
  {
    loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
    robot_model_ = loader_->getModel();


    ASSERT_TRUE(robot_model_ != nullptr);
    ASSERT_TRUE(kin.init(robot_model_->getJointModelGroup(GROUP_NAME)));
    ASSERT_TRUE(kin.calcFwdKin(VectorXd::Zero(6), homePose));
    ASSERT_NO_THROW(ik.init(kin));
    ASSERT_NO_THROW(planning_scene_.reset(new planning_scene::PlanningScene(robot_model_)));
    ASSERT_NO_THROW(config = ik.getSolverConfiguration());

    //Now assign collision detection plugin
    collision_detection::CollisionPluginLoader cd_loader;
    std::string class_name = "FCL";
    ASSERT_TRUE(cd_loader.activate(class_name, planning_scene_, true));
  }
};

/** @brief This tests the Constrained_IK init functions */
TEST_F(BasicIKTest, inputValidation)
{
  EXPECT_ANY_THROW(Constrained_IK().init(BasicKin()));
  EXPECT_NO_THROW(Constrained_IK().init(kin));

  EXPECT_EQ(Constrained_IK().checkInitialized(), constrained_ik::initialization_state::NothingInitialized);
  constrained_ik::Constraint *goal_ptr = new  constrained_ik::constraints::GoalPosition();
  ik.addConstraint(goal_ptr, constrained_ik::constraint_types::Primary);
  EXPECT_NE(ik.checkInitialized(), constrained_ik::initialization_state::NothingInitialized);
}

/** @brief This performs input validation test for the Constrained_IK calcInvKin function */
TEST_F(BasicIKTest, calcInvKinInputValidation)
{
  VectorXd seed = VectorXd::Zero(6);
  VectorXd joints;
  constrained_ik::Constraint *goal_ptr = new  constrained_ik::constraints::GoalPosition();
  ik.addConstraint(goal_ptr, constrained_ik::constraint_types::Primary);
  EXPECT_NE(ik.checkInitialized(), constrained_ik::initialization_state::NothingInitialized);
  EXPECT_ANY_THROW(Constrained_IK().calcInvKin(Affine3d::Identity(), seed, joints));      // un-init Constrained_IK
  EXPECT_ANY_THROW(ik.calcInvKin(Affine3d(Eigen::Matrix4d::Zero()), seed, joints)); // empty Pose (zeros in matrix because unitary rotation matrix is often in memory)
  EXPECT_ANY_THROW(ik.calcInvKin(homePose, VectorXd(), joints));                    // un-init Seed
  EXPECT_ANY_THROW(ik.calcInvKin(homePose, VectorXd::Zero(99), joints));            // wrong-sized Seed

  // valid input
  //   - GTest doesn't print exception info.  This method allows us to print that info, for easier troubleshooting
  try 
  {
    ik.calcInvKin(homePose, seed, joints);
    SUCCEED();
  } catch (const std::exception &ex) 
  {
    ADD_FAILURE() << ex.what();
  }
}

/** @brief This tests the Constrained_IK calcInvKin function against known poses using only primary constraint */
TEST_F(BasicIKTest, knownPoses)
{
  Affine3d pose, rslt_pose;
  VectorXd seed, expected(6), joints;
  ik.loadDefaultSolverConfiguration();
  config = ik.getSolverConfiguration();

  // seed position *at* expected solution
  expected << M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2;
  seed = expected;
  EXPECT_TRUE(kin.calcFwdKin(expected, pose)); // expect the pose to remain unchanged

  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
    "*at* from expected: " << expected.transpose() << std::endl;

  // adding primary constraints to move to a pose
  ik.clearConstraintList();
  constrained_ik::Constraint *goal_pose_ptr = new  constrained_ik::constraints::GoalPose();
  ik.addConstraint(goal_pose_ptr, constrained_ik::constraint_types::Primary);

  // seed = expected, pose = fwdK(expected)
  EXPECT_NO_THROW(ik.calcInvKin(pose, seed, joints));
  std::cout << "joint values returned" << joints << std::endl;
  EXPECT_TRUE(joints.isApprox(expected, 1e-10));

  // seed position *near* expected solution
  seed = expected + 0.01 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *near* #2
  seed = expected + 0.05 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // seed position *near* expected solution
  seed = expected + 0.1 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // test with seed far from expected position
  expected << M_PI_2, 0, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*far1* from expected: " << expected.transpose() << std::endl;
  config.primary_gain = 0.5;
  config.limit_primary_motion = false;
  ik.setSolverConfiguration(config);
  EXPECT_TRUE(ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *far* #2
  expected << 0, -M_PI_2, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*far2* from expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *farther*
  expected << M_PI_4, -M_PI_4, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*farther* from expected: " << expected.transpose() << std::endl;
  seed = VectorXd::Zero(expected.size());
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *very far*
  expected << M_PI_2, -M_PI_2, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*very far* from expected: " << expected.transpose() << std::endl;
  seed = VectorXd::Zero(expected.size());
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));
}

/** @brief This tests the Constrained_IK calcInvKin function null space motion */
TEST_F(BasicIKTest, NullMotion)
{
  Affine3d pose, rslt_pose;
  VectorXd seed, expected(6), joints;
  boost::random::mt19937 rng;  
  ik.loadDefaultSolverConfiguration();
  config = ik.getSolverConfiguration();
  config.limit_auxiliary_interations = false;
  config.limit_auxiliary_motion = false;
  config.limit_primary_motion = false;
  ik.setSolverConfiguration(config);

  // adding primary constraints to move to a position, and auxillary constraints to move to pose
  ik.clearConstraintList();
  constrained_ik::Constraint *goal_position_ptr = new  constrained_ik::constraints::GoalPosition();
  ik.addConstraint(goal_position_ptr, constrained_ik::constraint_types::Primary);
  constrained_ik::Constraint *goal_orientation_ptr = new  constrained_ik::constraints::GoalOrientation();
  ik.addConstraint(goal_orientation_ptr, constrained_ik::constraint_types::Auxiliary);

  // seed position *at* expected solution
  expected << M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2;

  kin.calcFwdKin(expected, pose);
  config.primary_gain = 0.5;
  config.auxiliary_gain = 0.5;
  ik.setSolverConfiguration(config);
  seed = expected;
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*at* from expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, seed, joints);
  EXPECT_TRUE(joints.isApprox(expected, 1e-10));

  // seed position *near* expected solution
  expected << M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2;
  kin.calcFwdKin(expected, pose);
  seed = expected + 0.01 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *near* #2
  for(int j=0; j<3; j++) // do 3 random examples of near starts
  {
    for(int i=0; i<6; i++)  // find a random pose
    {
      boost::random::uniform_int_distribution<int> angle_degrees(-177, 177) ;
      boost::random::uniform_int_distribution<int> small_angle_degrees(-3, 3) ;
      expected[i] = angle_degrees(rng)* 3.14/180.0;
      seed[i] = expected[i] + small_angle_degrees(rng)*3.14/180.0;
    }
    kin.calcFwdKin(expected, pose);
    config.primary_gain = 0.15;
    config.auxiliary_gain = 0.05;
    ik.setSolverConfiguration(config);
    EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
    EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
    EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
    EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));
  }

  // *far*
  int num_success=0;
  for(int j=0; j<20; j++) // do 3 random examples of far starts
  {
    for(int i=0; i<6; i++)  // find a random pose
    {
      boost::random::uniform_int_distribution<int> angle_degrees(-180+50, 180-50) ;
      boost::random::uniform_int_distribution<int> small_angle_degrees(-50, 50) ;
      expected[i] = angle_degrees(rng)* 3.14/180.0;
      seed[i] = expected[i] + small_angle_degrees(rng)*3.14/180.0;
    }
    kin.calcFwdKin(expected, pose);
    config.primary_gain = 0.15;
    config.auxiliary_gain = 0.05;
    ik.setSolverConfiguration(config);
    ik.calcInvKin(pose, seed, joints);
    kin.calcFwdKin(joints, rslt_pose);
    if(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3) && rslt_pose.translation().isApprox(pose.translation(), 1e-3)) num_success++;
  }
  EXPECT_GE(num_success, 15);
  
  // *farther*
  num_success=0;
  for(int j=0; j<20; j++) // do 20 random examples of farther starts
  {
    for(int i=0; i<6; i++)  // find a random pose
    {
      boost::random::uniform_int_distribution<int> angle_degrees(-180+90, 180-90) ;
      boost::random::uniform_int_distribution<int> small_angle_degrees(-90, 90) ;
      expected[i] = angle_degrees(rng)* 3.14/180.0;
      seed[i] = expected[i] + small_angle_degrees(rng)*3.14/180.0;
    }
    kin.calcFwdKin(expected, pose);
    config.primary_gain = 0.15;
    config.auxiliary_gain = 0.05;
    ik.setSolverConfiguration(config);
    ik.calcInvKin(pose, seed, joints);
    kin.calcFwdKin(joints, rslt_pose);
    if(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3) && rslt_pose.translation().isApprox(pose.translation(), 1e-3)) num_success++;
  }
  EXPECT_GE(num_success, 10);
}

/** @brief This tests the Constrained_IK calcInvKin function null space motion convergence for known poses*/
TEST_F(BasicIKTest, NullMotionPose)
{
  Affine3d pose, rslt_pose;
  VectorXd seed, expected(6), joints;
  ik.loadDefaultSolverConfiguration();
  config = ik.getSolverConfiguration();

  // adding primary constraints to move to an orientation, and auxillary constraints to move to position
  ik.clearConstraintList();
  constrained_ik::Constraint *goal_orientation_ptr = new  constrained_ik::constraints::GoalOrientation();
  ik.addConstraint(goal_orientation_ptr, constrained_ik::constraint_types::Primary);
  constrained_ik::Constraint *goal_position_ptr = new  constrained_ik::constraints::GoalPosition();
  ik.addConstraint(goal_position_ptr, constrained_ik::constraint_types::Auxiliary);

  // seed position *at* expected solution
  expected << M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2;
  kin.calcFwdKin(expected, pose);
  seed = expected;
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*at* from expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // seed position *near* expected solution
  seed = expected + 0.01 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));
  // *near* #2
  seed = expected + 0.05 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // seed position *near* expected solution
  seed = expected + 0.1 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // test with seed far from expected position
  expected << M_PI_2, 0, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*far1* from expected: " << expected.transpose() << std::endl;
  config.primary_gain = 0.1;
  config.auxiliary_gain = 0.1;
  config.limit_auxiliary_interations = false;
  config.limit_auxiliary_motion = false;
  config.limit_primary_motion = false;
  ik.setSolverConfiguration(config);
  seed = VectorXd::Zero(expected.size());
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *far* #2
  expected << 0, -M_PI_2, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*far2* from expected: " << expected.transpose() << std::endl;
  seed = VectorXd::Zero(expected.size());
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *farther*
  expected << M_PI_4, -M_PI_4, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*farther* from expected: " << expected.transpose() << std::endl;
  seed = VectorXd::Zero(expected.size());
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));

  // *very far*
  expected << M_PI_2, -M_PI_2, 0, 0, 0, 0;
  kin.calcFwdKin(expected, pose);
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*very far* from expected: " << expected.transpose() << std::endl;
  config.primary_gain = 0.1;
  config.auxiliary_gain = 0.001;
  config.solver_max_iterations = 10000;
  ik.setSolverConfiguration(config);
  seed = VectorXd::Zero(expected.size());
  EXPECT_TRUE(ik.calcInvKin(pose, seed, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints, rslt_pose));
  EXPECT_TRUE(rslt_pose.rotation().isApprox(pose.rotation(), 9e-3));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));
}

/**
 * @brief This test the AvoidObstacles constraint
 * The solver is setup with a primary GoalPosition constraint and
 * a axuiliary AvoidObstacles constraint. I then seeds the solver
 * with it current pose with a goal position as the current position
 * and the axuiliary constraint should move the robot in joint space
 * away from its current joint positions while maintaining the starting
 * position of the tcp.
 */
TEST_F(BasicIKTest, obstacleAvoidanceAuxiliaryConstraint)
{
  Affine3d pose, rslt_pose;
  VectorXd seed, expected(6), joints;
  ik.loadDefaultSolverConfiguration();
  config = ik.getSolverConfiguration();
  config.solver_min_iterations = 1;
  config.limit_auxiliary_interations = true;
  ik.setSolverConfiguration(config);

  // adding primary constraints to move to an orientation, and Auxiliary constraints to move to position
  ik.clearConstraintList();
  constrained_ik::Constraint *goal_position_ptr = new  constrained_ik::constraints::GoalPosition();

  std::vector<std::string> link_names;
  ik.getLinkNames(link_names);
  int link_id = link_names.size()/2;
  std::vector<std::string> obstacle_avoidance_link_name;
  constrained_ik::constraints::AvoidObstacles *avoid_obstacles_ptr =  new constrained_ik::constraints::AvoidObstacles();

  obstacle_avoidance_link_name.push_back(link_names[link_id]);
  avoid_obstacles_ptr->setAvoidanceLinks(obstacle_avoidance_link_name);
  avoid_obstacles_ptr->setMinDistance(link_names[link_id], 0.01);
  avoid_obstacles_ptr->setAmplitude(link_names[link_id], 1.0);
  avoid_obstacles_ptr->setAvoidanceDistance(link_names[link_id], 1.0);

  // add the constraints
  ik.addConstraint(goal_position_ptr, constrained_ik::constraint_types::Primary);
  ik.addConstraint(avoid_obstacles_ptr, constrained_ik::constraint_types::Auxiliary);

  // perform inverse kinematics but the position should not change, just the orientation
  expected << M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2;
  kin.calcFwdKin(expected, pose);
  seed = expected;
  config.auxiliary_max_iterations = 1;
  ik.setSolverConfiguration(config);
  EXPECT_TRUE(ik.calcInvKin(pose, seed, planning_scene_, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints,rslt_pose));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));
  EXPECT_NE(expected, joints);

  // do it again, but his time allow 100 iterations on the auxiliary constriants
  // this allows more obstacle avoidance after the position has converged
  config.auxiliary_max_iterations = 100;
  ik.setSolverConfiguration(config);
  EXPECT_TRUE(ik.calcInvKin(pose, seed, planning_scene_, joints));
  EXPECT_TRUE(kin.calcFwdKin(joints,rslt_pose));
  EXPECT_TRUE(rslt_pose.translation().isApprox(pose.translation(), 1e-3));
  EXPECT_NE(expected, joints);
}

/**
 * @brief This test checks the consistancy of the axisAngle calculations from a quaternion value
 * Namely does it always return an angle in +/-pi range? (The answer should be yes)
 */
TEST_F(BasicIKTest, consistancy)
{
    Eigen::Vector3d k(Eigen::Vector3d::UnitZ());
    for (double theta=-7.0; theta<7.0; theta+=1.0)
    {
        Eigen::Vector4d expected; expected << k, ik.rangedAngle(theta);

        //create angle-axis representation of theta @ k
        Eigen::AngleAxisd aa(theta, k);
        Eigen::Quaterniond q(aa);
        Eigen::AngleAxisd aa_q(q);

        //stuff results into vectorrd for direct comparison
        Eigen::Vector4d rslt; rslt << aa_q.axis(), aa_q.angle();
        rslt(3) = ik.rangedAngle(aa_q.angle());

        //if theta ~= 0, angleAxis/Quaternion operations will give [1,0,0] as vector
        if (fabs(theta) < 1e-6)
        {
            expected(0) = 1;
            expected(1) = 0;
            expected(2) = 0;
        }

        //rslt may be reverse vector & angle, which is still a valid representation
        EXPECT_TRUE(rslt.isApprox(expected, 1e-5) or rslt.isApprox(-expected, 1e-5));
    }
}
/** @brief This executes all tests for the Constraine_IK Class and its constraints */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc,argv,"test_constrained_ik");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

