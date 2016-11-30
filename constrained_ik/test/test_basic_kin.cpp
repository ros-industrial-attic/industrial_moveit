/**
 * @file test_basic_kin.cpp
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
#include <constrained_ik/basic_kin.h>
#include <boost/assign/list_of.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <boost/random/uniform_int_distribution.hpp>

using constrained_ik::basic_kin::BasicKin;
using Eigen::MatrixXd;
using Eigen::VectorXd;

const std::string GROUP_NAME = "manipulator"; /**< Default group name for tests */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

/**
 * @brief Test Fixtures
 * Consolidate variable-definitions and init functions for use by multiple tests.
 */
class BaseTest : public :: testing::Test
{
protected:
  BasicKin kin; /**< Basic Kinematic Model of the robot.  */
};

/** @brief The RobotTest Test Fixture Class*/
class RobotTest : public BaseTest
{
protected:

  robot_model_loader::RobotModelLoaderPtr loader_; /**< Used to load the robot model */
  moveit::core::RobotModelPtr robot_model_; /**< Robot model */

  /** @brief See base class for documention */
  virtual void SetUp()
  {

    loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
    robot_model_ = loader_->getModel();

    ASSERT_TRUE(robot_model_ != nullptr);
    ASSERT_TRUE(kin.init(robot_model_->getJointModelGroup(GROUP_NAME)));
  }

  /**
   * @brief Compares two poses to see if they are with in the provided tolerance
   * @param actual pose
   * @param expected pose
   * @param tol tolerance allowed
   * @return True if the difference is smaller than the tolerance, otherwise false
   */
  bool comparePoses(const std::vector<KDL::Frame> &actual, const std::vector<KDL::Frame> &expected, const double tol = 1e-6)
  {
      bool rtn;

      if (actual.size() != expected.size())
      {
        ROS_ERROR("comparePoses, number of poses different");
        return false;
      }
      for (size_t ii=0; ii<actual.size(); ++ii)
      {
        if (!KDL::Equal(actual[ii], expected[ii], tol))
        {
          ROS_ERROR("failure at pose %d",(int) ii);
          return false;
        }
      }
      return true;
  }
};

/** @brief The PInvTest Test Fixture Class */
class PInvTest : public RobotTest
{
protected:
  /**
   * @brief Used to test basic kin damped pseudo inverse function
   * @param rows number of rows
   * @param cols number of columns
   * @return True if correct, otherwise false
   */
  bool test_random(int rows, int cols)
  {
    MatrixXd A = MatrixXd::Random(rows, cols);
    VectorXd x = VectorXd::Random(cols);
    VectorXd b = A*x;
    VectorXd rslt;

    bool status = kin.solvePInv(A, b, rslt);
    return status && rslt.isApprox(x, 1e-10);
  }
};

/** @brief This tests the BasicKin init function */
TEST_F(RobotTest, inputValidation)
{
  EXPECT_FALSE(kin.init(NULL));
  EXPECT_TRUE(kin.init(robot_model_->getJointModelGroup(GROUP_NAME)));
}

/** @brief This performs input validation test for the BasicKin linkTransforms function */
TEST_F(RobotTest, linkTransformsInputValidation)
{
    std::vector<std::string> link_names = boost::assign::list_of("shoulder_link")("upper_arm_link")("forearm_link")("wrist_1_link")("wrist_2_link")("wrist_3_link");
    std::vector<KDL::Frame> poses;
    int n_links = link_names.size();

    EXPECT_FALSE(BasicKin().linkTransforms(VectorXd(), poses));                                     // un-init BasicKin, names, & Jnts
    EXPECT_FALSE(BasicKin().linkTransforms(VectorXd(), poses, std::vector<std::string>()));         // un-init BasicKin, names, & Jnts
    EXPECT_FALSE(BasicKin().linkTransforms(VectorXd(), poses, link_names));                         // un-init BasicKin & Jnts
    EXPECT_FALSE(BasicKin().linkTransforms(VectorXd::Zero(6), poses, std::vector<std::string>()));  // un-init BasicKin & names
    EXPECT_FALSE(BasicKin().linkTransforms(VectorXd::Zero(6), poses, link_names));                  // un-init BasicKin

    EXPECT_FALSE(kin.linkTransforms(VectorXd(), poses, std::vector<std::string>()));                // empty names & joints
    EXPECT_FALSE(kin.linkTransforms(VectorXd::Zero(99), poses, link_names));                        // too many joints
    EXPECT_FALSE(kin.linkTransforms(VectorXd::Constant(6, 1e10), poses, link_names));               // joints out-of-range

    EXPECT_TRUE(kin.linkTransforms(VectorXd::Zero(6), poses, link_names));                          // valid input
    EXPECT_TRUE(kin.linkTransforms(VectorXd::Zero(6), poses));                                      // valid input

    std::vector<std::string> link_names_short = boost::assign::list_of(link_names[5]);
    EXPECT_TRUE(kin.linkTransforms(VectorXd::Zero(6), poses, link_names_short));                    //valid short input

    link_names[5] = "fail_on_this";
    EXPECT_FALSE(kin.linkTransforms(VectorXd::Zero(6), poses, link_names));                         // invalid link list
    link_names_short[0] = link_names[5];
    EXPECT_FALSE(kin.linkTransforms(VectorXd::Zero(6), poses, link_names_short));                   // invalid & short link list
}

/** @brief This tests the BasicKin linkTransforms function against known poses */
TEST_F(RobotTest, linkTransformsKnownPoses)
{
    using KDL::Rotation;
    using KDL::Vector;
    using KDL::Frame;

    std::vector<std::string> link_names = boost::assign::list_of("shoulder_link")("upper_arm_link")("forearm_link")("wrist_1_link")("wrist_2_link")("wrist_3_link");
    VectorXd joint_angles(6);
    std::vector<Frame> actual, expected(6);

    //0,0,0,0,0,0
    joint_angles = VectorXd::Zero(6);
    expected[0] = Frame(KDL::Rotation::Quaternion(0.0, 0.0 , 0.0 , 1.0), Vector(0.000000, 0.000000, 0.127300));              
    expected[1] = Frame(KDL::Rotation::Quaternion(0.0, 0.707107, 0.0, 0.707107), Vector(0.000000, 0.220941, 0.127300));
    expected[2] = Frame(KDL::Rotation::Quaternion(0.0, 0.707107, 0.0, 0.707107), Vector(0.612000, 0.049041, 0.127300));
    expected[3] = Frame(KDL::Rotation::Quaternion(0.0, 1.0, 0.0, 0.0), Vector(1.184300, 0.049041, 0.127300));
    expected[4] = Frame(KDL::Rotation::Quaternion(0.0, 1.0, 0.0, 0.0), Vector(1.184300, 0.163941, 0.127300));
    expected[5] = Frame(KDL::Rotation::Quaternion(0.0, 1.0, 0.0, 0.0), Vector(1.184300, 0.163941, 0.011600));
    kin.linkTransforms(joint_angles, actual, link_names);
    
    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual, link_names));  //all link names
    EXPECT_TRUE(comparePoses(actual, expected, 1e-4));
    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual));              //no link names (defaults to all)
    EXPECT_GT(actual.size(), expected.size());
    actual.pop_back(); //for comparison remove ee_link which is not in list of test links.
    EXPECT_TRUE(comparePoses(actual, expected, 1e-4));

    //90,0,0,0,0,0
    joint_angles(0) = M_PI_2;
    expected[0] = Frame(KDL::Rotation::Quaternion(0.0, 0.0 , 0.707107, 0.707107), Vector(0.000000, 0.000000, 0.127300));              
    expected[1] = Frame(KDL::Rotation::Quaternion(-0.5, 0.5, 0.5, 0.5), Vector(-0.220941, 0.000000, 0.127300));
    expected[2] = Frame(KDL::Rotation::Quaternion(-0.5, 0.5, 0.5, 0.5), Vector(-0.049041, 0.612000, 0.127300));
    expected[3] = Frame(KDL::Rotation::Quaternion(-0.707107, 0.707107, 0.0, 0.0), Vector(-0.049041, 1.184300, 0.127300));
    expected[4] = Frame(KDL::Rotation::Quaternion(-0.707107, 0.707107, 0.0, 0.0), Vector(-0.163941, 1.184300, 0.127300));
    expected[5] = Frame(KDL::Rotation::Quaternion(-0.707107, 0.707107, 0.0, 0.0), Vector(-0.163941, 1.184300, 0.011600));
    kin.linkTransforms(joint_angles, actual, link_names);

    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual, link_names));  //all link names
    EXPECT_TRUE(comparePoses(actual, expected, 1e-4));
    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual));              //no link names (defaults to all)
    EXPECT_GT(actual.size(), expected.size());
    actual.pop_back(); //for comparison remove ee_link which is not in list of test links.
    EXPECT_TRUE(comparePoses(actual, expected, 1e-4));

    //0,-90,0,0,0,0
    joint_angles(0) = 0.;
    joint_angles(1) = -M_PI_2;
    expected[0] = Frame(KDL::Rotation::Quaternion(0.000000, 0.000000, 0.000000, 1.000000), Vector(0.000000, 0.000000, 0.127300));              
    expected[1] = Frame(KDL::Rotation::Quaternion(0.000000, -0.000000, 0.000000, 1.000000), Vector(0.000000, 0.220941, 0.127300));              
    expected[2] = Frame(KDL::Rotation::Quaternion(0.000000, -0.000000, 0.000000, 1.000000), Vector(-0.000000, 0.049041, 0.739300));              
    expected[3] = Frame(KDL::Rotation::Quaternion(0.000000, 0.707107, 0.000000, 0.707107), Vector(-0.000000, 0.049041, 1.311600));              
    expected[4] = Frame(KDL::Rotation::Quaternion(0.000000, 0.707107, 0.000000, 0.707107), Vector(-0.000000, 0.163941, 1.311600));              
    expected[5] = Frame(KDL::Rotation::Quaternion(0.000000, 0.707107, 0.000000, 0.707107), Vector(0.115700, 0.163941, 1.311600));              
    kin.linkTransforms(joint_angles, actual, link_names);
    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual, link_names));  //all link names
    EXPECT_TRUE(comparePoses(actual, expected, 1e-4));
    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual));              //no link names (defaults to all)
    EXPECT_GT(actual.size(), expected.size());
    actual.pop_back(); //for comparison remove ee_link which is not in list of test links.
    EXPECT_TRUE(comparePoses(actual, expected, 1e-4));

    //joints 2-5
    joint_angles = VectorXd::Zero(6);
    std::vector<std::string> link_names_short(link_names.begin()+1, link_names.end()-1);
    expected[1] = Frame(KDL::Rotation::Quaternion(0.000000, 0.707107, 0.000000, 0.707107), Vector(0.000000, 0.220941, 0.127300));
    expected[2] = Frame(KDL::Rotation::Quaternion(0.000000, 0.707107, 0.000000, 0.707107), Vector(0.612000, 0.049041, 0.127300));
    expected[3] = Frame(KDL::Rotation::Quaternion(0.000000, 1.000000, 0.000000, 0.000000), Vector(1.184300, 0.049041, 0.127300));
    expected[4] = Frame(KDL::Rotation::Quaternion(0.000000, 1.000000, 0.000000, 0.000000), Vector(1.184300, 0.163941, 0.127300));
    std::vector<Frame> expected_short(expected.begin()+1, expected.end()-1);
    kin.linkTransforms(joint_angles, actual, link_names);

    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual, link_names_short));    //all link names
    EXPECT_TRUE(comparePoses(actual, expected_short, 1e-4));
    EXPECT_TRUE(kin.linkTransforms(joint_angles, actual));                      //no link names (defaults to all)
    EXPECT_GT(actual.size(), expected.size());
    actual.pop_back(); //for comparison remove ee_link which is not in list of test links.
    EXPECT_FALSE(comparePoses(actual, expected, 1e-4));
}

/** @brief This performs input validation for the BasicKin calcFwdKin function */
TEST_F(RobotTest, calcFwdKinInputValidation)
{
  //test for calcFwdKin(joints, pose)
  Eigen::Affine3d pose;

  EXPECT_FALSE(BasicKin().calcFwdKin(VectorXd(), pose));            // un-init BasicKin & Jnts
  EXPECT_FALSE(BasicKin().calcFwdKin(VectorXd::Zero(6), pose));     // un-init BasicKin
  EXPECT_FALSE(kin.calcFwdKin(VectorXd(), pose));                   // empty joints
  EXPECT_FALSE(kin.calcFwdKin(VectorXd::Zero(99), pose));           // too many joints
  EXPECT_FALSE(kin.calcFwdKin(VectorXd::Constant(6, 1e10), pose));  // joints out-of-range

  EXPECT_TRUE(kin.calcFwdKin(VectorXd::Zero(6), pose));             // valid input
}

/** @brief This tests the BasicKin calcFwdKin function against known poses */
TEST_F(RobotTest, calcFwdKinKnownPoses)
{
  VectorXd joints = VectorXd::Zero(6);
  Eigen::Affine3d expected, result;
  double base_to_tip;
  double base_to_tip_expected;
  // all joints 0
  EXPECT_TRUE(kin.calcFwdKin(joints, result));
  double x = result.translation().x();
  double y = result.translation().y();
  double z = result.translation().z();
  base_to_tip_expected  = x*x + y*y + z*z;

  // 1st joint 90 deg,all others 0
  joints(0) = M_PI_2;
  EXPECT_TRUE(kin.calcFwdKin(joints, result));
  x = result.translation().x();
  y = result.translation().y();
  z = result.translation().z();
  base_to_tip = x*x + y*y + z*z;
  EXPECT_NEAR(base_to_tip, base_to_tip_expected, 1e-3);

  // last joint -90, all others 0
  joints(0) = 0.0;
  joints(5) = -M_PI_2;
  EXPECT_TRUE(kin.calcFwdKin(joints, result));
  x = result.translation().x();
  y = result.translation().y();
  z = result.translation().z();
  base_to_tip = x*x + y*y + z*z;
  EXPECT_NEAR(base_to_tip, base_to_tip_expected, 1e-3);

}

/** @brief This performs input validation for the BasicKin calcJacobian function */
TEST_F(RobotTest, calcJacobianInputValidation)
{
    Eigen::MatrixXd jacobian;
    EXPECT_FALSE(BasicKin().calcJacobian(VectorXd(), jacobian));            // un-init BasicKin & Jnts
    EXPECT_FALSE(BasicKin().calcJacobian(VectorXd::Zero(6), jacobian));     // un-init BasicKin
    EXPECT_FALSE(kin.calcJacobian(VectorXd(), jacobian));                   // empty joints
    EXPECT_FALSE(kin.calcJacobian(VectorXd::Zero(99), jacobian));           // too many joints
    EXPECT_FALSE(kin.calcJacobian(VectorXd::Constant(6, 1e10), jacobian));  // joints out-of-range

    EXPECT_TRUE(kin.calcJacobian(VectorXd::Zero(6), jacobian));             // valid input
}

/** @brief This tests the BasicKin calcJacobian function against known poses */
TEST_F(RobotTest, calcJacobianKnownPoses)
{
  VectorXd joints = VectorXd(6);
  VectorXd updated_joints = VectorXd(6);
  MatrixXd jacobian;
  Eigen::Affine3d pose;
  boost::random::mt19937 rng;  
  double delta = 1e-3;
  
  // use random joint states, compute the numerical Jacobian and compare to that computed
  // all joints at a random pose
  for(int j=0; j<10; j++)// try 10 different poses
  {
    for(int i=0; i<(int)joints.size(); i++)  // find a random pose
    {
      boost::random::uniform_int_distribution<int> angle_degrees(-180, 180) ;
      joints[i] = angle_degrees(rng)* 3.14/180.0;
    }
    kin.calcFwdKin(joints,pose);
    EXPECT_TRUE(kin.calcJacobian(joints, jacobian));
    for(int i=0; i<(int) joints.size(); i++)
    {
      updated_joints = joints;
      updated_joints[i] += delta;
      Eigen::Affine3d updated_pose;
      kin.calcFwdKin(updated_joints, updated_pose);
      double delta_x = (updated_pose.translation().x() - pose.translation().x())/delta;
      double delta_y = (updated_pose.translation().y() - pose.translation().y())/delta;
      double delta_z = (updated_pose.translation().z() - pose.translation().z())/delta;
      EXPECT_NEAR(delta_x, jacobian(0,i), 1e-3);
      EXPECT_NEAR(delta_y, jacobian(1,i), 1e-3);
      EXPECT_NEAR(delta_z, jacobian(2,i), 1e-3);
      Eigen::AngleAxisd r12(pose.rotation().transpose()*updated_pose.rotation());   // rotation from p1 -> p2
      double theta = r12.angle();          // TODO: move rangedAngle to utils class
      theta = copysign(fmod(fabs(theta),2.0*M_PI), theta);
      if (theta < -M_PI) theta = theta+2.*M_PI;
      if (theta > M_PI)  theta = theta-2.*M_PI;
      Eigen::VectorXd omega = (pose.rotation() * r12.axis() * theta)/delta;                        // axis k * theta expressed in frame0
      EXPECT_NEAR(omega(0), jacobian(3,i), 1e-3);
      EXPECT_NEAR(omega(1), jacobian(4,i), 1e-3);
      EXPECT_NEAR(omega(2), jacobian(5,i), 1e-3);
    }
  }
}

/** @brief This performs input validation for the BasicKin solvePInv function */
TEST_F(PInvTest, solvePInvInputValidation)
{
  VectorXd vResult;

  EXPECT_FALSE(kin.solvePInv(MatrixXd(), VectorXd(), vResult)); // will generate ROS_ERROR("Empty matrices not supported in solvePinv()");
  EXPECT_FALSE(kin.solvePInv(MatrixXd(6,3), VectorXd(1), vResult)); // will generate ROS_ERROR("Matrix size mismatch:...");
  EXPECT_TRUE(kin.solvePInv(MatrixXd::Zero(6,3), VectorXd::Zero(6), vResult));
  EXPECT_TRUE(kin.solvePInv(MatrixXd::Zero(3,6), VectorXd::Zero(3), vResult));
  EXPECT_TRUE(kin.solvePInv(MatrixXd::Zero(6,6), VectorXd::Zero(6), vResult));
}

/** @brief This test exercises the BasicKin solvePInv function with random inputs */
TEST_F(PInvTest, solvePInvRandomInputs)
{
  VectorXd vResult;

  // Test square matrices
  for (int i=0; i<100; ++i)
    ASSERT_TRUE(test_random(10, 10));

  // Test over-constrained matrices
  for (int i=0; i<100; ++i)
    ASSERT_TRUE(test_random(10, 5));
}

/** @brief This executes all tests for the BasicKin Class */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc,argv,"test_BasicKin");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

