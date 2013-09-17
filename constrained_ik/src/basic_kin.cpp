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


#include "constrained_ik/basic_kin.h"
#include <ros/ros.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>


namespace constrained_ik
{
namespace basic_kin
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool BasicKin::calcAllFwdKin(const VectorXd &joint_angles, std::vector<KDL::Frame> &poses) const
{
    int n = joint_angles.size();
    KDL::JntArray kdl_joints;

    if (!checkInitialized()) return false;
    if (!checkJoints(joint_angles)) return false;

    EigenToKDL(joint_angles, kdl_joints);

    // run FK solver
    poses.resize(n);
    for (unsigned int ii=0; ii<n; ++ii)
    {
        if (fk_solver_->JntToCart(kdl_joints, poses[ii], ii) < 0)
        {
            ROS_ERROR_STREAM("Failed to calculate FK for joint " << n);
            return false;
        }
    }
    return true;
}

bool BasicKin::calcFwdKin(const VectorXd &joint_angles, Eigen::Affine3d &pose) const
{
//  int n = joint_angles.size();
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

bool BasicKin::calcFwdKin(  const VectorXd &joint_angles,
                            const std::string &base,
                            const std::string &tip,
                            KDL::Frame &pose)
{
    KDL::Chain chain;
    if (!kdl_tree_.getChain(base, tip, chain))
    {
      ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                       base << "' and '" << tip <<"'");
      return false;
    }

    if (joint_angles.size() != chain.getNrOfJoints())
    {
        ROS_ERROR_STREAM("Number of joint angles [" << joint_angles.size() <<
                            "] must match number of joints [" << chain.getNrOfJoints() << "].");
        return false;
    }

    subchain_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain));

    KDL::JntArray joints;
    joints.data = joint_angles;
    if (subchain_fk_solver_->JntToCart(joints, pose) < 0)
        return false;
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
  joints.data = vec;
//  joints.resize(vec.size());
//
//  for (int i=0; i<vec.size(); ++i)
//    joints(i) = vec[i];
}

bool BasicKin::getJointNames(std::vector<std::string> &names) const
{
    if (!initialized_)
    {
        ROS_ERROR("Kinematics must be initialized before retrieving joint names");
        return false;
    }
    names = joint_list_;
    return true;
}

bool BasicKin::getJointNames(const KDL::Chain &chain, std::vector<std::string> &names) const
{
    if (!initialized_)
    {
        ROS_ERROR("Kinematics must be initialized before retrieving joint names");
        return false;
    }

    unsigned int n = chain.getNrOfJoints();
    names.resize(n);

    KDL::Joint joint;
    for (unsigned int ii=0; ii<n; ++ii)
    {
        joint = chain.getSegment(ii).getJoint();
        if (joint.getType() != joint.None)
            names[ii] = joint.getName();
//        if (joint.getTypeName()[0] != 'R')
//            ROS_ERROR_STREAM("Joint should be of type 'rotary', found type " << joint.getTypeName() << " at position " << ii);
    }
    return true;
}

bool BasicKin::getLinkNames(std::vector<std::string> &names) const
{
    if (!initialized_)
    {
        ROS_ERROR("Kinematics must be initialized before retrieving link names");
        return false;
    }
    names = link_list_;
    return true;
}

bool BasicKin::getLinkNames(const KDL::Chain &chain, std::vector<std::string> &names) const
{
    if (!initialized_)
    {
        ROS_ERROR("Kinematics must be initialized before retrieving link names");
        return false;
    }

    unsigned int n = chain.getNrOfSegments();
    names.resize(n);

    for (unsigned int ii=0; ii<n; ++ii)
    {
        names[ii] = robot_chain_.getSegment(ii).getName();
    }
    return true;
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

//  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(robot, kdl_tree_))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return false;
  }

  if (!kdl_tree_.getChain(base_name, tip_name, robot_chain_))
  {
    ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                     base_name << "' and '" << tip_name <<"'");
    return false;
  }

  joint_list_.resize(robot_chain_.getNrOfJoints());
  link_list_.resize(robot_chain_.getNrOfSegments());
  joint_limits_.resize(robot_chain_.getNrOfJoints(), 2);

  for (int i=0, j=0; i<robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment &seg = robot_chain_.getSegment(i);
    link_list_[i] = seg.getName();

    const KDL::Joint   &jnt = seg.getJoint();
    if (jnt.getType() == KDL::Joint::None) continue;

    joint_list_[j] = jnt.getTypeName();
    joint_limits_(j,0) = robot.getJoint(jnt.getName())->limits->lower;
    joint_limits_(j,1) = robot.getJoint(jnt.getName())->limits->upper;
    j++;
  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  initialized_ = true;
  return true;
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

BasicKin& BasicKin::operator=(const BasicKin& rhs)
{
  initialized_  = rhs.initialized_;
  robot_chain_  = rhs.robot_chain_;
  joint_limits_ = rhs.joint_limits_;
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  return *this;
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
    ROS_ERROR("Matrix size mismatch: A(%ld,%ld), b(%ld)",
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

} // namespace basic_kin
} // namespace constrained_ik

