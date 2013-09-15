/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

