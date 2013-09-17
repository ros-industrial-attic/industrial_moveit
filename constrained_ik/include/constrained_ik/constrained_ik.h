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


#ifndef CONSTRAINED_IK_H
#define CONSTRAINED_IK_H

#include "constrained_ik/basic_kin.h"
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

  bool getJointNames(std::vector<std::string> &names) const {return kin_.getJointNames(names);};
  bool getLinkNames(std::vector<std::string> &names) const {return kin_.getLinkNames(names);};
  unsigned int numJoints() const {return kin_.numJoints();};
  bool calcAllFwdKin(const Eigen::VectorXd &joints, std::vector<KDL::Frame> &poses) const {return kin_.calcAllFwdKin(joints, poses);};

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

