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
#include "constraint_group.h"
#include "solver_state.h"
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
  virtual ~Constrained_IK() { }

  //TODO document
  bool linkTransforms(const Eigen::VectorXd &joints,
                      std::vector<KDL::Frame> &poses,
                      const std::vector<std::string> link_names = std::vector<std::string>()) const
  {return kin_.linkTransforms(joints, poses, link_names);};

  /**@brief Add a new constraint to this IK solver
   * @param constraint Constraint to limit IK solution
   */
  virtual void addConstraint(Constraint* constraint) { constraints_.add(constraint); }

  //TODO document
  virtual void calcInvKin(const Eigen::Affine3d &pose, const Eigen::VectorXd &joint_seed, Eigen::VectorXd &joint_angles);

  /**@brief Checks to see if object is initialized (ie: init() has been called)
   * @return True if object is initialized
   */
  bool checkInitialized() const { return initialized_ && !constraints_.empty(); }

  /**@brief Delete all constraints
   */
  void clearConstraintList();

  /**@brief Getter for joint names
   * @param names Output vector of strings naming all joints in robot
   * @return True if BasicKin object is initialized
   */
  bool getJointNames(std::vector<std::string> &names) const {return kin_.getJointNames(names);};

  /**@brief Getter for joint_update_gain_ (gain used to scale joint diff in IK loop)
   * @return Value of joint_update_gain_
   */
  inline double getJointUpdateGain() const {return joint_update_gain_;};

  /**@brief Getter for joint_convergence_tol_ (convergence criteria in IK loop)
   * Used to check if solution is progressing or has settled
   * @return Value of joint_convergence_tol_
   */
  inline double getJtCnvTolerance() const {return joint_convergence_tol_;};

  /**@brief Getter for kinematics object
   * @return Reference to active kinematics object
   */
  inline const basic_kin::BasicKin& getKin() const {return kin_;}

  /**@brief Getter for link names
   * @param names Output vector of strings naming all links in robot
   * @return True is BasicKin object is initialized
   */
  bool getLinkNames(std::vector<std::string> &names) const {return kin_.getLinkNames(names);};

  /**@brief Getter for max_iter_ (maximum allowable iterations in IK loop)
   * @return Value of max_iter_
   */
  inline unsigned int getMaxIter() const {return max_iter_;};

  /**@brief Getter for latest solver state
   * @return Latest solver state
   */
  inline const SolverState& getState() const { return state_; }

  /**@brief Initializes object with robot info
   * @param robot Robot urdf information
   * @param base_name Name of base link
   * @param tip_name Name of tip link
   */
  void init(const urdf::Model &robot, const std::string &base_name, const std::string &tip_name);

  /**@brief Initializes object with kinematic model of robot
   * @param kin BasicKin object with robot info
   */
  virtual void init(const basic_kin::BasicKin &kin);

  /**@brief Getter for BasicKin numJoints
   * @return Number of variable joints in robot
   */
  unsigned int numJoints() const {return kin_.numJoints();};

  /**@brief Translates an angle to lie between +/-PI
   * @param angle Input angle, radians
   * @return Output angle within range of +/- PI
   */
  static double rangedAngle(double angle);

  /**@brief Setter for joint_update_gain_ (gain used to scale joint diff in IK loop)
   * @param gain New value for joint_update_gain_
   */
  inline void setJointUpdateGain(const double gain) {joint_update_gain_ = gain;};

  /**@brief Setter for joint_convergence_tol_ (convergence criteria in IK loop)
   * @param jt_cnv_tol new value for joint_convergence_tol_
   */
  inline void setJtCnvTolerance(const double jt_cnv_tol) {joint_convergence_tol_ = jt_cnv_tol;};

  /**@brief Setter for man_iter_ (maximum allowable iterations in IK loop)
   * @param max_iter New value for max_iter_
   */
  inline void setMaxIter(const unsigned int max_iter) {max_iter_ = max_iter;};

protected:
  // gains and scaling factors
  double joint_update_gain_;

  // termination-criteria limits / tolerances
  unsigned int max_iter_;
  double joint_convergence_tol_;

  // constraints
  ConstraintGroup constraints_;

  // state/counter data
  bool initialized_;
  SolverState state_;
  basic_kin::BasicKin kin_;

  bool debug_;
  std::vector<Eigen::VectorXd> iteration_path_;

  /**@brief Pure definition for calculating constraint error
   * @return Error vector (b-input in calcPInv)
   */
  virtual Eigen::VectorXd calcConstraintError();

  /**@brief Pure definition for calculating Jacobian
   * @return Jacobian matrix (A-input in calcPInv)
   */
  virtual Eigen::MatrixXd calcConstraintJacobian();

  //TODO document
  void clipToJointLimits(Eigen::VectorXd &joints);

  //TODO document
  virtual bool checkStatus() const;

  //TODO document
  virtual void reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);

  //TODO document
  virtual void update(const Eigen::VectorXd &joints);

}; // class Constrained_IK

} // namespace constrained_ik

#endif // CONSTRAINED_IK_H

