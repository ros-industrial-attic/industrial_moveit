/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <constrained_ik/basic_ik.h>

using constrained_ik::basic_ik::Basic_IK;
using constrained_ik::basic_kin::BasicKin;
using Eigen::Affine3d;
using Eigen::VectorXd;

/* ----------------------------------------------------------------
 * Test Fixtures
 *   consolidate variable-definitions and init functions
 *   for use by multiple tests.
 * ----------------------------------------------------------------
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

  EXPECT_FALSE(Basic_IK().checkInitialized());
  EXPECT_TRUE(ik.checkInitialized());
}

TEST_F(calcInvKin, inputValidation)
{
  VectorXd seed = VectorXd::Zero(6);
  VectorXd joints;

//  try {
//      Affine3d btest0, btest2(Eigen::Matrix4d::Zero());
//      ik.calcInvKin(btest2, seed, joints);
//      ADD_FAILURE();
//  } catch (const std::exception &ex) {
//      SUCCEED();
//  }

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

//  pose = Eigen::Translation3d(-0.17035, 0, 1.14911) * Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5);
  ik.calcInvKin(pose, expected, joints);
  EXPECT_TRUE(joints.isApprox(expected, 1e-10));

  // seed position *near* expected solution
  seed = expected + 0.01 * VectorXd::Random(expected.size());
//  std::cout << "Testing seed vector: " << seed.transpose() << std::endl;
  ik.calcInvKin(pose, seed, joints);
  EXPECT_TRUE(joints.isApprox(expected, 0.01));
  kin.calcFwdKin(joints, rslt_pose);
//  std::cout << rslt_pose.matrix() << std::endl << pose.matrix() << std::endl;
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  //TODO check orientations of 0, pi/2, pi, -pi/2, -pi?

//  // test with seed far from expected position
  expected << M_PI_2, 0, 0, 0, 0, 0;
  pose.translation().matrix() << -0.1245, 0.4115, 1.1058;
  pose.matrix().topLeftCorner(3,3) << 0,-1, 0,
                                      1, 0, 0,
                                      0, 0, 1;
//  std::cout << "Testing expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints);
  kin.calcFwdKin(joints, rslt_pose);
//  std::cout << rslt_pose.matrix() << std::endl << pose.matrix() << std::endl;
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  expected << 0, -M_PI_2, 0, 0, 0, 0;
  pose.translation().matrix() << -0.4318, 0.1245, 1.0855;
  pose.matrix().topLeftCorner(3,3) << 0, 0,-1,
                                      0, 1, 0,
                                      1, 0, 0;
//  std::cout << "Testing expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints);
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  expected << M_PI_4, -M_PI_4, 0, 0, 0, 0;
  pose.translation().matrix() << -0.0982, 0.0778, 1.2703;
  pose.matrix().topLeftCorner(3,3) << 0.5, -0.707107,-0.5,
                                      0.5,  0.707107,-0.5,
                                      0.707107, 0,    0.707107;
//  std::cout << "Testing expected: " << expected.transpose() << std::endl;
  EXPECT_ANY_THROW( ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints) );
//  kin.calcFwdKin(joints, rslt_pose);
//  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));

  expected << M_PI_2, -M_PI_2, 0, 0, 0, 0;
  pose.translation().matrix() << -0.1245, -0.4318, 1.0855;
  pose.matrix().topLeftCorner(3,3) << 0, -1, 0,
                                      0,  0,-1,
                                      1,  0, 0;
  std::cout << "Testing expected: " << expected.transpose() << std::endl;
  ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints);
//  EXPECT_ANY_THROW( ik.calcInvKin(pose, VectorXd::Zero(expected.size()), joints) );
  kin.calcFwdKin(joints, rslt_pose);
  EXPECT_TRUE(rslt_pose.isApprox(pose, 0.005));
}

TEST_F(axisAngleCheck, consistancy)
{
    Eigen::Vector3d k(Eigen::Vector3d::UnitZ());
    for (double theta=-7.0; theta<7.0; theta+=1.0)
    {
//        std::cout << "in: " << theta << ", out: " << ik.rangedAngle(theta) << std::endl;
        Eigen::Vector4d expected; expected << k, ik.rangedAngle(theta);
//        std::cout << "testing " << k.transpose() << " " << theta << std::endl;
        Eigen::AngleAxisd aa(theta, k);
        Eigen::Quaterniond q(aa);
        Eigen::AngleAxisd aa_q(q);
        Eigen::Vector4d rslt; rslt << aa_q.axis(), aa_q.angle();
        rslt(3) = ik.rangedAngle(aa_q.angle());
        expected(3) = ik.rangedAngle(theta);
        if (fabs(theta) < 1e-6)
        {
            expected(0) = 1;
            expected(2) = 0;
        }
        EXPECT_TRUE(rslt.isApprox(expected, 1e-5) or rslt.isApprox(-expected, 1e-5));
//        std::cout << rslt.transpose() << std::endl;
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

