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


#ifndef BASIC_IK_H
#define BASIC_IK_H

#include "constrained_ik.h"

namespace constrained_ik
{
namespace basic_ik
{

/**
 * \brief Basic IK Solver
 *          - solve for 6DOF cartesian goal
 */
class Basic_IK : public Constrained_IK
{
public:
  Basic_IK();
  ~Basic_IK() {};

protected:
  // termination-criteria limits / tolerances
  double pos_err_tol_;
  double rot_err_tol_;

  // state/counter data
  Eigen::Affine3d pose_;
  double pos_err_;
  double rot_err_;

  virtual Eigen::MatrixXd calcConstraintJacobian();
  virtual Eigen::VectorXd calcConstraintError();

  virtual void reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);
  virtual void update(const Eigen::VectorXd &joints);
  virtual bool checkStatus() const;

  static double calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);
  static double calcAngle(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);
  static Eigen::Vector3d calcAngleError(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);
  static Eigen::Vector3d calcAngleError2(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);
}; // class Basic_IK


} // namespace basic_ik
} // namespace constrained_ik


#endif // BASIC_IK_H

