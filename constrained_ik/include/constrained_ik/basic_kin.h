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

  //TODO document
  bool calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose) const;

  //TODO document
  //TODO test
  bool calcFwdKin(const Eigen::VectorXd &joint_angles,
                  const std::string &base,
                  const std::string &tip,
                  KDL::Frame &pose);

  //TODO document
  bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian) const;

  //TODO document
  bool checkInitialized() const { return initialized_; }

  //TODO document
  //TODO test
  bool checkJoints(const Eigen::VectorXd &vec) const;

  //TODO document
  //TODO test
  bool getJointNames(std::vector<std::string> &names) const;

  //TODO document
  //TODO test
  bool getJointNames(const KDL::Chain &chain, std::vector<std::string> &names) const;

  //TODO document
  Eigen::MatrixXd getLimits() const { return joint_limits_; }

  //TODO document
  //TODO test
  bool getLinkNames(std::vector<std::string> &names) const;

  //TODO document
  //TODO test
  bool getLinkNames(const KDL::Chain &chain, std::vector<std::string> &names) const;

  //TODO document
  bool init(const urdf::Model &robot, const std::string &base_name, const std::string &tip_name);

  //TODO document
  unsigned int numJoints() const { return robot_chain_.getNrOfJoints(); }

  //TODO comment
  bool linkTransforms(const Eigen::VectorXd &joint_angles,
                      std::vector<KDL::Frame> &poses,
                      const std::vector<std::string> &link_names = std::vector<std::string>()) const;

  //TODO document
  BasicKin& operator=(const BasicKin& rhs);

  //TODO document
  bool solvePInv(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x) const;

private:
  bool initialized_;
  KDL::Chain  robot_chain_;
  KDL::Tree   kdl_tree_;
  std::vector<std::string> joint_list_, link_list_;
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_limits_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_, subchain_fk_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  //TODO document
  static void EigenToKDL(const Eigen::VectorXd &vec, KDL::JntArray &joints);

  //TODO comment
  int getJointNum(const std::string &joint_name) const;

  //TODO comment
  int getLinkNum(const std::string &link_name) const;

  //TODO document
  static void KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform);

  //TODO document
  static void KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix);

}; // class BasicKin

} // namespace basic_kin
} // namespace constrained_ik


#endif // BASIC_KIN_H

