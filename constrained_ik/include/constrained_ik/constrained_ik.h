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


#ifndef CONSTRAINED_IK_H
#define CONSTRAINED_IK_H

#include "basic_kin.h"
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <urdf/model.h>

namespace constrained_ik
{

/**
 * \brief Damped Least-Squares Inverse Kinematic Solution
 *          - see derived classes for more complex constrained-IK solvers
 */
class Constrained_IK
{
public:
  Constrained_IK();
  virtual ~Constrained_IK() {};

  void init(const urdf::Model &robot, const std::string &base_name, const std::string &tip_name);
  virtual void init(const basic_kin::BasicKin &kin);
  bool checkInitialized() const { return initialized_; }

  virtual void calcInvKin(const Eigen::Affine3d &pose, const Eigen::VectorXd &joint_seed, Eigen::VectorXd &joint_angles);

  inline void setJointUpdateGain(const double gain) {joint_update_gain_ = gain;};
  inline void setMaxIter(const int max_iter) {max_iter_ = max_iter;};
  inline void setJtCnvTolerance(const double jt_cnv_tol) {joint_convergence_tol_ = jt_cnv_tol;};

  inline double getJointUpdateGain() const {return joint_update_gain_;};
  inline int getMaxIter() const {return max_iter_;};
  inline double getJtCnvTolerance() const {return joint_convergence_tol_;};

  static double rangedAngle(double angle);

protected:
  // gains and scaling factors
  double joint_update_gain_;

  // termination-criteria limits / tolerances
  int    max_iter_;
  double joint_convergence_tol_;

  // state/counter data
  int iter_;
  Eigen::VectorXd joints_;
  Eigen::VectorXd joints_delta_;

  bool initialized_;
  basic_kin::BasicKin kin_;
  Eigen::Affine3d goal_;
  Eigen::VectorXd joint_seed_;

  bool debug_;
  std::vector<Eigen::VectorXd> iteration_path_;

  virtual Eigen::MatrixXd calcConstraintJacobian()=0;
  virtual Eigen::VectorXd calcConstraintError()=0;

  virtual void reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);
  virtual void update(const Eigen::VectorXd &joints);
  virtual bool checkStatus() const;

  void clipToJointLimits(Eigen::VectorXd &joints);

}; // class Constrained_IK

} // namespace constrained_ik

#endif // CONSTRAINED_IK_H

