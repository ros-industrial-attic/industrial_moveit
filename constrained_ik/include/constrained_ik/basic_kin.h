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


#ifndef BASIC_KIN_H
#define BASIC_KIN_H

#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>

namespace constrained_ik
{
namespace basic_kin
{

/**
 * \brief Basic low-level kinematics functions.
 *        Typically, just wrappers around the equivalent KDL calls.
 *
 */
class BasicKin
{
public:
  BasicKin() : initialized_(false) {};
  ~BasicKin() {};

  BasicKin& operator=(const BasicKin& rhs);

  bool init(const urdf::Model &robot, const std::string &base_name, const std::string &tip_name);

  bool calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose) const;

  bool calcFwdKin(const Eigen::VectorXd &joint_angles, const std::string &base, const std::string &tip, KDL::Frame &pose);

  bool calcAllFwdKin(const Eigen::VectorXd &joint_angles, std::vector<KDL::Frame> &poses) const;

  bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian) const;

  bool solvePInv(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x) const;

  bool checkInitialized() const { return initialized_; }
  bool checkJoints(const Eigen::VectorXd &vec) const;

  unsigned int numJoints() const { return robot_chain_.getNrOfJoints(); }
  Eigen::MatrixXd getLimits() const { return joint_limits_; }

  bool getJointNames(std::vector<std::string> &names) const;
  bool getLinkNames(std::vector<std::string> &names) const;

private:
  bool initialized_;
  KDL::Chain  robot_chain_;
  KDL::Tree   kdl_tree_;
  Eigen::MatrixXd joint_limits_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_, subchain_fk_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  static void EigenToKDL(const Eigen::VectorXd &vec, KDL::JntArray &joints);
  static void KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform);
  static void KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix);

}; // class BasicKin

} // namespace basic_kin
} // namespace constrained_ik


#endif // BASIC_KIN_H

