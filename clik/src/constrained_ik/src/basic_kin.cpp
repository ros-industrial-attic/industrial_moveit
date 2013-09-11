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

#include "constrained_ik/basic_kin.h"
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>

namespace constrained_ik
{
namespace basic_kin
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

BasicKin& BasicKin::operator=(const BasicKin& rhs)
{
  initialized_  = rhs.initialized_;
  robot_chain_  = rhs.robot_chain_;
  joint_limits_ = rhs.joint_limits_;
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  return *this;
}

bool BasicKin::init(const urdf::Model &robot,
                   const std::string &base_name, const std::string &tip_name)
{
  initialized_ = false;

  if (!robot.getRoot())
  {
    ROS_ERROR("Invalid URDF in BasicKin::init call");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(robot, tree))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return false;
  }

  if (!tree.getChain(base_name, tip_name, robot_chain_))
  {
    ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                     base_name << "' and '" << tip_name <<"'");
    return false;
  }

  joint_limits_.resize(robot_chain_.getNrOfJoints(), 2);
  for (int i=0, j=0; i<robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment &seg = robot_chain_.getSegment(i);
    const KDL::Joint   &jnt = seg.getJoint();
    if (jnt.getType() == KDL::Joint::None) continue;

    joint_limits_(j,0) = robot.getJoint(jnt.getName())->limits->lower;
    joint_limits_(j,1) = robot.getJoint(jnt.getName())->limits->upper;
    j++;
  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  initialized_ = true;
  return true;
}

bool BasicKin::calcFwdKin(const VectorXd &joint_angles, Eigen::Affine3d &pose) const
{
  int n = joint_angles.size();
  KDL::JntArray kdl_joints;

  if (!checkInitialized()) return false;
  if (!checkJoints(joint_angles)) return false;

  EigenToKDL(joint_angles, kdl_joints);

  // run FK solver
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose) < 0)
  {
    ROS_ERROR("Failed to calculate FK");
    return false;
  }

  KDLToEigen(kdl_pose, pose);
  return true;
}

bool BasicKin::calcJacobian(const VectorXd &joint_angles, MatrixXd &jacobian) const
{
  KDL::JntArray kdl_joints;

  if (!checkInitialized()) return false;
  if (!checkJoints(joint_angles)) return false;

  EigenToKDL(joint_angles, kdl_joints);

  // compute jacobian
  KDL::Jacobian kdl_jacobian(joint_angles.size());
  jac_solver_->JntToJac(kdl_joints, kdl_jacobian);

  KDLToEigen(kdl_jacobian, jacobian);
  return true;
}

bool BasicKin::solvePInv(const MatrixXd &A, const VectorXd &b, VectorXd &x) const
{
  const double eps = 0.00001;  // TODO: Turn into class member var
  const double lambda = 0.01;  // TODO: Turn into class member var

  if ( (A.rows() == 0) || (A.cols() == 0) )
  {
    ROS_ERROR("Empty matrices not supported");
    return false;
  }

  if ( A.rows() != b.size() )
  {
    ROS_ERROR("Matrix size mismatch: A(%d,%d), b(%d)",
              A.rows(), A.cols(), b.size());
    return false;
  }

  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const MatrixXd &U = svd.matrixU();
  const VectorXd &Sv = svd.singularValues();
  const MatrixXd &V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  int nSv = Sv.size();
  VectorXd inv_Sv(nSv);
  for(int i=0; i<nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
      inv_Sv(i) = 1/Sv(i);
    else
      inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
  }

  x = V * inv_Sv.asDiagonal() * U.transpose() * b;

  return true;
}

// check for consistency in # and limits of joints
bool BasicKin::checkJoints(const VectorXd &vec) const
{
  if (vec.size() != robot_chain_.getNrOfJoints())
  {
    ROS_ERROR("Number of joint angles (%d) don't match robot_model (%d)",
              (int)vec.size(), robot_chain_.getNrOfJoints());
    return false;
  }

  bool jnt_bounds_ok = true;
  for (int i=0; i<vec.size(); ++i)
    if ( (vec[i] < joint_limits_(i,0)) || (vec(i) > joint_limits_(i,1)) )
    {
      ROS_ERROR("Joint %d is out-of-range (%g < %g < %g)",
                i, joint_limits_(i,0), vec(i), joint_limits_(i,1));
      jnt_bounds_ok = false;
    }
  if (jnt_bounds_ok == false) return false;

  return true;
}

void BasicKin::EigenToKDL(const VectorXd &vec, KDL::JntArray &joints)
{
  joints.resize(vec.size());

  for (int i=0; i<vec.size(); ++i)
    joints(i) = vec[i];
}

void BasicKin::KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform)
{
  transform.setIdentity();

  // translation
  for (size_t i=0; i<3; ++i)
    transform(i,3) = frame.p[i];

  // rotation matrix
  for (size_t i=0; i<9; ++i)
    transform(i/3, i%3) = frame.M.data[i];
}

void BasicKin::KDLToEigen(const KDL::Jacobian &jacobian, MatrixXd &matrix)
{
  matrix.resize(jacobian.rows(), jacobian.columns());

  for (size_t i=0; i<jacobian.rows(); ++i)
    for (size_t j=0; j<jacobian.columns(); ++j)
      matrix(i,j) = jacobian(i,j);
}


} // namespace basic_kin
} // namespace constrained_ik

