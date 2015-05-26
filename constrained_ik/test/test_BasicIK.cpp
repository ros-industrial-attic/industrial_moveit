/**
* @file test_BasicIK.cpp
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
* @license Software License Agreement (Apache License)\n
* \n
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at\n
* \n
* http://www.apache.org/licenses/LICENSE-2.0\n
* \n
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <constrained_ik/ik/basic_ik.h>

using constrained_ik::basic_ik::Basic_IK;
using constrained_ik::basic_kin::BasicKin;
using Eigen::Affine3d;
using Eigen::VectorXd;

/**
 * @brief Test Fixtures
 * Consolidate variable-definitions and init functions for use by multiple tests.
 */
class BasicIKTest : public ::testing::Test
{
protected:
  urdf::Model model;
  BasicKin kin;
  Basic_IK ik;
  Affine3d homePose;

  virtual void SetUp()
  {
    ASSERT_TRUE(model.initFile(urdf_file));
    ASSERT_TRUE(kin.init(model, "base_link", "link_6"));
    ASSERT_TRUE(kin.calcFwdKin(VectorXd::Zero(6), homePose));
    ASSERT_NO_THROW(ik.init(kin));
  }
private:
  static const std::string urdf_file;
};
const std::string BasicIKTest::urdf_file = "puma_560.urdf";


typedef BasicIKTest init;
typedef BasicIKTest calcInvKin;
typedef BasicIKTest axisAngleCheck;
/* ---------------------------------------------------------------- */

TEST_F(init, inputValidation)
{
  EXPECT_ANY_THROW(Basic_IK().init(urdf::Model(), "", ""));
  EXPECT_NO_THROW(Basic_IK().init(model, "base_link", "link_6"));
  EXPECT_ANY_THROW(Basic_IK().init(BasicKin()));
  EXPECT_NO_THROW(Basic_IK().init(kin));

  EXPECT_FALSE(Basic_IK().checkInitialized(constrained_ik::constraint_types::primary));
  EXPECT_TRUE(ik.checkInitialized(constrained_ik::constraint_types::primary));
}

TEST_F(calcInvKin, inputValidation)
{
  VectorXd seed = VectorXd::Zero(6);
  VectorXd joints;

  EXPECT_TRUE(ik.checkInitialized(constrained_ik::constraint_types::primary));
  EXPECT_ANY_THROW(Basic_IK().calcInvKin(Affine3d::Identity(), seed, joints));      // un-init Basic_IK
  EXPECT_ANY_THROW(ik.calcInvKin(Affine3d(Eigen::Matrix4d::Zero()), seed, joints)); // empty Pose (zeros in matrix because unitary rotation matrix is often in memory)
  EXPECT_ANY_THROW(ik.calcInvKin(homePose, VectorXd(), joints));                    // un-init Seed
  EXPECT_ANY_THROW(ik.calcInvKin(homePose, VectorXd::Zero(99), joints));            // wrong-sized Seed

  // valid input
  //   - GTest doesn't print exception info.  This method allows us to print that info, for easier troubleshooting
  try {
    ik.calcInvKin(homePose, seed, joints);
    SUCCEED();
  } catch (const std::exception &ex) {
    ADD_FAILURE() << ex.what();
  }
}

TEST_F(calcInvKin, knownPoses)
{
  Affine3d pose, rslt_pose;
  VectorXd seed, expected(6), joints;

  // seed position *at* expected solution
  expected << M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_2;
  pose.translation().matrix() << -0.1245, 0.0203, 0.6740;
  pose.matrix().topLeftCorner(3,3) << 0,0,1,
                                      1,0,0,
                                      0,1,0;
  seed = expected;
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*at* from expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, seed, joints);
  EXPECT_TRUE(joints.isApprox(expected, 1e-10));

  // seed position *near* expected solution
  seed = expected + 0.01 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, seed, joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  // *near* #2
  seed = expected + 0.05 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, seed, joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  // seed position *near* expected solution
  seed = expected + 0.1 * VectorXd::Random(expected.size());
  std::cout << "Testing seed vector " << seed.transpose() << std::endl <<
               "*near* from expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, seed, joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  // test with seed far from expected position
  expected << M_PI_2, 0, 0, 0, 0, 0;
  pose.translation().matrix() << -0.1245, 0.4115, 1.1058;
  pose.matrix().topLeftCorner(3,3) << 0,-1, 0,
                                      1, 0, 0,
                                      0, 0, 1;
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*far* from expected: " << expected.transpose() << std::endl;
  ik.setPrimaryKp(.4);
  ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  // *far* #2
  expected << 0, -M_PI_2, 0, 0, 0, 0;
  pose.translation().matrix() << -0.4318, 0.1245, 1.0855;
  pose.matrix().topLeftCorner(3,3) << 0, 0,-1,
                                      0, 1, 0,
                                      1, 0, 0;
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*far* from expected: " << expected.transpose() << std::endl;
  
  ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  // *farther*
  expected << M_PI_4, -M_PI_4, 0, 0, 0, 0;
  pose.translation().matrix() << -0.0982, 0.0778, 1.2703;
  pose.matrix().topLeftCorner(3,3) << 0.5, -0.707107,-0.5,
                                      0.5,  0.707107,-0.5,
                                      0.707107, 0,    0.707107;
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*farther* from expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  // *very far*
  expected << M_PI_2, -M_PI_2, 0, 0, 0, 0;
  pose.translation().matrix() << -0.1245, -0.4318, 1.0855;
  pose.matrix().topLeftCorner(3,3) << 0, -1, 0,
                                      0,  0,-1,
                                      1,  0, 0;
  std::cout << "Testing seed vector " << VectorXd::Zero(expected.size()).transpose() << std::endl <<
               "*very far* from expected: " << expected.transpose() << std::endl;
  ik.setPrimaryKp(.1);
  ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_FALSE(rslt_pose.isApprox(pose, 0.005));
}

/*This test checks the consistancy of the axisAngle calculations from a quaternion value,
  namely does it always return an angle in +/-pi range?
  (The answer should be yes)
  */
TEST_F(axisAngleCheck, consistancy)
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

