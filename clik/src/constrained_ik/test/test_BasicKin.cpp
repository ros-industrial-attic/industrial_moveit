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
#include <constrained_ik/basic_kin.h>

using constrained_ik::basic_kin::BasicKin;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* ----------------------------------------------------------------
 * Test Fixtures
 *   consolidate variable-definitions and init functions
 *   for use by multiple tests.
 * ----------------------------------------------------------------
 */
class BaseTest : public :: testing::Test
{
protected:
  BasicKin kin;
};

class RobotTest : public BaseTest
{
protected:
  urdf::Model model;

  virtual void SetUp()
  {
    ASSERT_TRUE(model.initFile(urdf_file));
    ASSERT_TRUE(kin.init(model, "base_link", "link_6"));
  }

private:
  static const std::string urdf_file;
};
const std::string RobotTest::urdf_file = "puma_560.urdf";

class PInvTest : public BaseTest
{
protected:
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

typedef RobotTest init;
typedef RobotTest calcFwdKin;
typedef RobotTest calcJacobian;
typedef PInvTest  solvePInv;
/* ---------------------------------------------------------------- */

TEST_F(init, inputValidation)
{
  EXPECT_FALSE(kin.init(urdf::Model(), "base_link", "tip_link"));
  EXPECT_FALSE(kin.init(model, "", ""));
  EXPECT_FALSE(kin.init(model, "base_link", ""));
  EXPECT_FALSE(kin.init(model, "", "link_6"));
  EXPECT_FALSE(kin.init(model, "INVALID", "link_6"));
  EXPECT_FALSE(kin.init(model, "base_link", "INVALID"));
  EXPECT_TRUE(kin.init(model, "base_link", "link_6"));
}


TEST_F(calcFwdKin, inputValidation)
{
  Eigen::Affine3d pose;

  EXPECT_FALSE(BasicKin().calcFwdKin(VectorXd(), pose));            // un-init BasicKin & Jnts
  EXPECT_FALSE(BasicKin().calcFwdKin(VectorXd::Zero(6), pose));     // un-init BasicKin
  EXPECT_FALSE(kin.calcFwdKin(VectorXd(), pose));                   // empty joints
  EXPECT_FALSE(kin.calcFwdKin(VectorXd::Zero(99), pose));           // too many joints
  EXPECT_FALSE(kin.calcFwdKin(VectorXd::Constant(6, 1e10), pose));  // joints out-of-range

  EXPECT_TRUE(kin.calcFwdKin(VectorXd::Zero(6), pose));             // valid input
}


TEST_F(calcFwdKin, knownPoses)
{
  VectorXd joints = VectorXd::Zero(6);
  Eigen::Affine3d expected, result;

  // all joints 0
  EXPECT_TRUE(kin.calcFwdKin(joints, result));
  expected = Eigen::Translation3d(0.41148, 0.12446, 1.1058);
  EXPECT_TRUE(result.isApprox(expected, 1e-3));

  // 1st joint 90 deg,all others 0
  joints(0) = M_PI_2;
  EXPECT_TRUE(kin.calcFwdKin(joints, result));
  expected = Eigen::Translation3d(-0.12446, 0.41148, 1.1058);
  expected.rotate(Eigen::AngleAxis<double> (M_PI_2, Eigen::Vector3d(0,0,1)));
  EXPECT_TRUE(result.isApprox(expected, 1e-3));

  // 2nd joint -90, all others 0
  joints(0) = 0.0;
  joints(1) = -M_PI_2;
  EXPECT_TRUE(kin.calcFwdKin(joints, result));
  expected = Eigen::Translation3d(-0.4318, 0.12446, 1.08548);
  expected.rotate(Eigen::AngleAxis<double> (-M_PI_2, Eigen::Vector3d(0,1,0)));
  EXPECT_TRUE(result.isApprox(expected, 1e-3));
}


TEST_F(calcJacobian, inputValidation)
{
    Eigen::MatrixXd jacobian;
    EXPECT_FALSE(BasicKin().calcJacobian(VectorXd(), jacobian));            // un-init BasicKin & Jnts
    EXPECT_FALSE(BasicKin().calcJacobian(VectorXd::Zero(6), jacobian));     // un-init BasicKin
    EXPECT_FALSE(kin.calcJacobian(VectorXd(), jacobian));                   // empty joints
    EXPECT_FALSE(kin.calcJacobian(VectorXd::Zero(99), jacobian));           // too many joints
    EXPECT_FALSE(kin.calcJacobian(VectorXd::Constant(6, 1e10), jacobian));  // joints out-of-range

    EXPECT_TRUE(kin.calcJacobian(VectorXd::Zero(6), jacobian));             // valid input
}


TEST_F(calcJacobian, knownPoses)
{
    VectorXd joints = VectorXd::Zero(6);
    MatrixXd expected, result;

    // all joints 0
    EXPECT_TRUE(kin.calcJacobian(joints, result));
    expected.resize(6,6);
    expected << -0.12446, 0.4318,  0.4318, 0, 0, 0,
                 0.41148, 0,       0,      0, 0, 0,
                 0,      -0.41148, 0.0203, 0, 0, 0,
                 0,       0,       0,      0, 0, 0,
                 0,       1,       1,      0, 1, 0,
                 1,       0,       0,      1, 0, 1;
    EXPECT_TRUE(result.isApprox(expected, 1e-3));

    joints(0) = M_PI_2;
    EXPECT_TRUE(kin.calcJacobian(joints, result));
    expected << -0.41148, 0,       0,       0,  0, 0,
                -0.12446, 0.4318,  0.4318,  0,  0, 0,
                 0,      -0.41148, 0.02032, 0,  0, 0,
                 0,      -1,      -1,       0, -1, 0,
                 0,       0,       0,       0,  0, 0,
                 1,       0,       0,       1,  0, 1;
    EXPECT_TRUE(result.isApprox(expected, 1e-3));

    joints(1) = -M_PI_2;
    EXPECT_TRUE(kin.calcJacobian(joints, result));
    expected <<  0.4318,  0,        0,       0,  0, 0,
                -0.12446, 0.41148, -0.02032, 0,  0, 0,
                 0,       0.4318,   0.4318,  0,  0, 0,
                 0,      -1,       -1,       0, -1, 0,
                 0,       0,        0,      -1,  0, -1,
                 1,       0,        0,       0,  0, 0;
    EXPECT_TRUE(result.isApprox(expected, 1e-3));

    joints(2) = -M_PI_2;
    EXPECT_TRUE(kin.calcJacobian(joints, result));
    expected <<  -0.02032, 0,        0,       0,  0,  0,
                 -0.12446, 0,       -0.4318,  0,  0,  0,
                  0,      -0.02032, -0.02032, 0,  0,  0,
                  0,      -1,       -1,       0, -1,  0,
                  0,       0,        0,       0,  0,  0,
                  1,       0,        0,      -1,  0, -1;
    EXPECT_TRUE(result.isApprox(expected, 1e-3));
//        std::cout << joints << std::endl;
//        std::cout << "result: " <<std::endl << result << std::endl;
//        std::cout << "expected: " <<std::endl << expected << std::endl;

    joints(3) = M_PI_2;
    EXPECT_TRUE(kin.calcJacobian(joints, result));
    expected << -0.02032, 0,        0,       0, 0,  0,
                -0.12446, 0,       -0.4318,  0, 0,  0,
                 0,      -0.02032, -0.02032, 0, 0,  0,
                 0,      -1,       -1,       0, 0,  0,
                 0,       0,        0,       0, 1,  0,
                 1,       0,        0,      -1, 0, -1;
    EXPECT_TRUE(result.isApprox(expected, 1e-3));

    joints(4) = M_PI_2;
    EXPECT_TRUE(kin.calcJacobian(joints, result));
    expected << -0.02032, 0,        0,       0, 0,  0,
                -0.12446, 0,       -0.4318,  0, 0,  0,
                 0,      -0.02032, -0.02032, 0, 0,  0,
                 0,      -1,       -1,       0, 0, -1,
                 0,       0,        0,       0, 1,  0,
                 1,       0,        0,      -1, 0,  0;
    EXPECT_TRUE(result.isApprox(expected, 1e-3));

    joints(5) = M_PI_2;
    EXPECT_TRUE(kin.calcJacobian(joints, result));
    expected << -0.02032, 0,        0,       0, 0,  0,
                -0.12446, 0,       -0.4318,  0, 0,  0,
                 0,      -0.02032, -0.02032, 0, 0,  0,
                 0,      -1,       -1,       0, 0, -1,
                 0,       0,        0,       0, 1,  0,
                 1,       0,        0,      -1, 0,  0;
    EXPECT_TRUE(result.isApprox(expected, 1e-3));
}


TEST_F(solvePInv, inputValidation)
{
  VectorXd vResult;

  EXPECT_FALSE(kin.solvePInv(MatrixXd(), VectorXd(), vResult));
  EXPECT_FALSE(kin.solvePInv(MatrixXd(6,3), VectorXd(1), vResult));
  EXPECT_TRUE(kin.solvePInv(MatrixXd::Zero(6,3), VectorXd::Zero(6), vResult));
  EXPECT_TRUE(kin.solvePInv(MatrixXd::Zero(3,6), VectorXd::Zero(3), vResult));
  EXPECT_TRUE(kin.solvePInv(MatrixXd::Zero(6,6), VectorXd::Zero(6), vResult));
}


TEST_F(solvePInv, randomInputs)
{
  VectorXd vResult;

  // Test square matrices
  for (int i=0; i<100; ++i)
    ASSERT_TRUE(test_random(10, 10));

  // Test over-constrained matrices
  for (int i=0; i<100; ++i)
    ASSERT_TRUE(test_random(10, 5));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

