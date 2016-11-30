/**
 * @file basic_kin.cpp
 * @brief Basic low-level kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 * @author dsolomon
 * @date Sep 15, 2013
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
#include "constrained_ik/basic_kin.h"
#include <ros/ros.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/robot_model.h>
#include <urdf/model.h>


namespace constrained_ik
{
namespace basic_kin
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool BasicKin::calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose) const
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

bool BasicKin::calcFwdKin(const Eigen::VectorXd &joint_angles,
                          const std::string &base,
                          const std::string &tip,
                          KDL::Frame &pose) const
{
  // note, because the base and tip are different, fk_solver gets updated
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

    KDL::ChainFkSolverPos_recursive subchain_fk_solver(chain);

    KDL::JntArray joints;
    joints.data = joint_angles;
    if (subchain_fk_solver.JntToCart(joints, pose) < 0)
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

int BasicKin::getJointNum(const std::string &joint_name) const
{
    std::vector<std::string>::const_iterator it = find(joint_list_.begin(), joint_list_.end(), joint_name);
    if (it != joint_list_.end())
    {
        return it-joint_list_.begin();
    }
    return it-joint_list_.begin()+1;
}

int BasicKin::getLinkNum(const std::string &link_name) const
{
    std::vector<std::string>::const_iterator it = find(link_list_.begin(), link_list_.end(), link_name);
    if (it != link_list_.end())
    {
        return it-link_list_.begin();
    }
    return it-link_list_.begin()+1;
}

bool BasicKin::init(const moveit::core::JointModelGroup* group)
{
  initialized_ = false;

  if(group == NULL)
  {
    ROS_ERROR_STREAM("Null pointer to JointModelGroup");
    return false;
  }

  const robot_model::RobotModel& r  = group->getParentModel();
  const boost::shared_ptr<const urdf::ModelInterface> urdf = group->getParentModel().getURDF();
  base_name_ = group->getLinkModels().front()->getParentLinkModel()->getName();
  tip_name_ = group->getLinkModels().back()->getName();

  if (!urdf->getRoot())
  {
    ROS_ERROR("Invalid URDF in BasicKin::init call");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree_))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return false;
  }

  if (!kdl_tree_.getChain(base_name_, tip_name_, robot_chain_))
  {
    ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                     base_name_ << "' and '" << tip_name_ <<"'");
    return false;
  }

  joint_list_.resize(robot_chain_.getNrOfJoints());
  joint_limits_.resize(robot_chain_.getNrOfJoints(), 2);

  link_list_ = group->getLinkModelNames();

  for (int i=0, j=0; i<robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment &seg = robot_chain_.getSegment(i);
    const KDL::Joint   &jnt = seg.getJoint();
    if (jnt.getType() == KDL::Joint::None) continue;

    joint_list_[j] = jnt.getName();
    joint_limits_(j,0) = urdf->getJoint(jnt.getName())->limits->lower;
    joint_limits_(j,1) = urdf->getJoint(jnt.getName())->limits->upper;
    j++;
  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  initialized_ = true;
  group_ = group;

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

void BasicKin::KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix)
{
  matrix.resize(jacobian.rows(), jacobian.columns());

  for (size_t i=0; i<jacobian.rows(); ++i)
    for (size_t j=0; j<jacobian.columns(); ++j)
      matrix(i,j) = jacobian(i,j);
}

bool BasicKin::getSubChain(const std::string link_name, KDL::Chain &chain) const
{
  if (!kdl_tree_.getChain(base_name_, link_name, chain))
  {
    ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                     base_name_ << "' and '" << link_name <<"'");
    return false;
  }
  else
  {
    return true;
  }
}

bool BasicKin::linkTransforms(const VectorXd &joint_angles,
                              std::vector<KDL::Frame> &poses,
                              const std::vector<std::string> &link_names) const
{
  if (!checkInitialized())
  {
    ROS_ERROR("BasicKin not initialized in linkTransforms()");
    return false;
  }
  if (!checkJoints(joint_angles))
  {
    ROS_ERROR("BasicKin checkJoints failed in linkTransforms()");
    return false;
  }

    std::vector<std::string> links(link_names);
    size_t n = links.size();
    if (!n) /*if link_names is an empty vector, return transforms of all links*/
    {
        links = link_list_;
        n = links.size();
    }

    KDL::JntArray kdl_joints;
    EigenToKDL(joint_angles, kdl_joints);

    // run FK solver
    poses.resize(n);
    int link_num;
    for (size_t ii=0; ii<n; ++ii)
    {
        link_num = getLinkNum(links[ii]);
        if (fk_solver_->JntToCart(kdl_joints, poses[ii], link_num<0? -1:link_num+1) < 0) /*root=0, link1=1, therefore add +1 to link num*/
        {
            ROS_ERROR_STREAM("Failed to calculate FK for joint " << n);
            return false;
        }
    }
    return true;
}

BasicKin& BasicKin::operator=(const BasicKin& rhs)
{
  initialized_  = rhs.initialized_;
  robot_chain_  = rhs.robot_chain_;
  kdl_tree_ = rhs.kdl_tree_;
  joint_limits_ = rhs.joint_limits_;
  joint_list_ = rhs.joint_list_;
  link_list_ = rhs.link_list_;
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));
  group_ = rhs.group_;
  base_name_ = rhs.base_name_;
  tip_name_ = rhs.tip_name_;

  return *this;
}

bool BasicKin::solvePInv(const MatrixXd &A, const VectorXd &b, VectorXd &x) const
{
  const double eps = 0.00001;  // TODO: Turn into class member var
  const double lambda = 0.01;  // TODO: Turn into class member var

  if ( (A.rows() == 0) || (A.cols() == 0) )
  {
    ROS_ERROR("Empty matrices not supported in solvePinv()");
    return false;
  }

  if ( A.rows() != b.size() )
  {
    ROS_ERROR("Matrix size mismatch: A(%ld,%ld), b(%ld)",
              A.rows(), A.cols(), b.size());
    return false;
  }

  //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
  //in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const MatrixXd &U = svd.matrixU();
  const VectorXd &Sv = svd.singularValues();
  const MatrixXd &V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  // damp inverse with lambda so that inverse doesn't oscillate near solution
  size_t nSv = Sv.size();
  VectorXd inv_Sv(nSv);
  for(size_t i=0; i<nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
      inv_Sv(i) = 1/Sv(i);
    else
      inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
  }
  x = V * inv_Sv.asDiagonal() * U.transpose() * b;
  return true;
}

bool BasicKin::dampedPInv(const MatrixXd &A, MatrixXd &P, const double eps, const double lambda)
{
  if ( (A.rows() == 0) || (A.cols() == 0) )
  {
    ROS_ERROR("Empty matrices not supported in dampedPInv()");
    return false;
  }

  //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
  //in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const MatrixXd &U = svd.matrixU();
  const VectorXd &Sv = svd.singularValues();
  const MatrixXd &V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  // damp inverse with lambda so that inverse doesn't oscillate near solution
  size_t nSv = Sv.size();
  VectorXd inv_Sv(nSv);
  for(size_t i=0; i<nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
      inv_Sv(i) = 1/Sv(i);
    else
    {
      inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
    }
  }
  P = V * inv_Sv.asDiagonal() * U.transpose();
  return true;
}

} // namespace basic_kin
} // namespace constrained_ik

