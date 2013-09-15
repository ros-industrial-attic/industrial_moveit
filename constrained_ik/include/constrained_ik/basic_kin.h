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


#ifndef BASIC_KIN_H
#define BASIC_KIN_H

#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
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

  bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian) const;

  bool solvePInv(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x) const;

  bool checkInitialized() const { return initialized_; }
  bool checkJoints(const Eigen::VectorXd &vec) const;

  unsigned int numJoints() const { return robot_chain_.getNrOfJoints(); }
  Eigen::MatrixXd getLimits() const { return joint_limits_; }

private:
  bool initialized_;
  KDL::Chain  robot_chain_;
  Eigen::MatrixXd joint_limits_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  static void EigenToKDL(const Eigen::VectorXd &vec, KDL::JntArray &joints);
  static void KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform);
  static void KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix);

}; // class BasicKin

} // namespace basic_kin
} // namespace constrained_ik


#endif // BASIC_KIN_H

