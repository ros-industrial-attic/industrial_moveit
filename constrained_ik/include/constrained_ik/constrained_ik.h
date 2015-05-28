/**
 * @file constrained_ik.h
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
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
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
#include <constrained_ik/enum_types.h>
#include <moveit/planning_scene/planning_scene.h>

namespace constrained_ik
{

/**
 * @brief Damped Least-Squares Inverse Kinematic Solution
 *          - see derived classes for more complex constrained-IK solvers
 */
class Constrained_IK
{

public:
  Constrained_IK();
  virtual ~Constrained_IK() { }

  /**
   * @brief computes and returns the link transfoms of the named joints
   * @param joints a vector of joints
   * @param poses the desired pose transforms
   * @param link_names the names of the links
   * @return true on success
   */  
  bool linkTransforms(const Eigen::VectorXd &joints,
                      std::vector<KDL::Frame> &poses,
                      const std::vector<std::string> link_names = std::vector<std::string>()) const
  {return kin_.linkTransforms(joints, poses, link_names);}

  /**
   * @brief Add a new constraint to this IK solver
   * @param constraint Constraint to limit IK solution
   * @param constraint_type Contraint type (primary or auxiliary)
   */
  virtual void addConstraint(Constraint* constraint, constraint_types::ConstraintType constraint_type)
  {
    switch(constraint_type)
    {
      case constraint_types::primary:
        primary_constraints_.add(constraint);
        break;
      case constraint_types::auxiliary:
        auxiliary_constraints_.add(constraint);
        break;
    }
  }

  /**
   * @brief computes the inverse kinematics for the given pose of the tip link
   * @param pose  The pose of the tip link
   * @param joint_seed joint values that is used as the initial guess
   * @param planning_scene pointer to a planning scene that holds all the object in the environment.  Use by the solver to check for collision; if
   *            a null pointer is passed then collisions are ignored.
   * @param joint_angles The joint pose that places the tip link to the desired pose.
   */
  virtual void calcInvKin(const Eigen::Affine3d &pose, const Eigen::VectorXd &joint_seed,
                          planning_scene::PlanningSceneConstPtr planning_scene,
                          Eigen::VectorXd &joint_angles);

  /**
   * @brief computes the inverse kinematics for the given pose of the tip link
   * @param pose  The pose of the tip link
   * @param joint_seed joint values that is used as the initial guess
   * @param joint_angles The joint pose that places the tip link to the desired pose.
   */
  virtual void calcInvKin(const Eigen::Affine3d &pose, const Eigen::VectorXd &joint_seed,
                          Eigen::VectorXd &joint_angles);

  /**
   * @brief Checks to see if object is initialized (ie: init() has been called)
   * @param constraint_type Contraint type (primary or auxiliary)
   * @return True if object is initialized
   */
  bool checkInitialized(constraint_types::ConstraintType constraint_type) const
  {
    switch(constraint_type)
    {
      case constraint_types::primary:
        return initialized_ && !primary_constraints_.empty();
      case constraint_types::auxiliary:
        return initialized_ && !auxiliary_constraints_.empty();
    }
  }

  /** @brief Delete all constraints */
  void clearConstraintList();

  /**
   * @brief Getter for joint names
   * @param names Output vector of strings naming all joints in robot
   * @return True if BasicKin object is initialized
   */
  bool getJointNames(std::vector<std::string> &names) const {return kin_.getJointNames(names);}

  /**
   * @brief Getter for joint_convergence_tol_ (convergence criteria in IK loop)
   * Used to check if solution is progressing or has settled
   * @return Value of joint_convergence_tol_
   */
  inline double getJtCnvTolerance() const {return joint_convergence_tol_;}

  /**
   * @brief Getter for kinematics object
   * @return Reference to active kinematics object
   */
  inline const basic_kin::BasicKin& getKin() const {return kin_;}

  /**
   * @brief Getter for link names
   * @param names Output vector of strings naming all links in robot
   * @return True is BasicKin object is initialized
   */
  bool getLinkNames(std::vector<std::string> &names) const {return kin_.getLinkNames(names);}

  /**
   * @brief Getter for max_iter_ (maximum allowable iterations in IK loop)
   * @return Value of max_iter_
   */
  inline unsigned int getMaxIter() const {return max_iter_;}

  /**
   * @brief Getter for latest solver state
   * @return Latest solver state
   */
  inline const SolverState& getState() const { return state_; }

  /**
   * @brief Initializes object with kinematic model of robot
   * @param kin BasicKin object with robot info
   */
  virtual void init(const basic_kin::BasicKin &kin);

  /**
   * @brief Getter for BasicKin numJoints
   * @return Number of variable joints in robot
   */
  unsigned int numJoints() const {return kin_.numJoints();}

  /**
   * @brief Translates an angle to lie between +/-PI
   * @param angle Input angle, radians
   * @return Output angle within range of +/- PI
   */
  static double rangedAngle(double angle);

  /**
   * @brief Setter for joint_convergence_tol_ (convergence criteria in IK loop)
   * @param jt_cnv_tol new value for joint_convergence_tol_
   */
  inline void setJtCnvTolerance(const double jt_cnv_tol) {joint_convergence_tol_ = jt_cnv_tol;}

  /**
   * @brief Setter for man_iter_ (maximum allowable iterations in IK loop)
   * @param max_iter New value for max_iter_
   */
  inline void setMaxIter(const unsigned int max_iter) {max_iter_ = max_iter;}

  /**
   * @brief Setter for primary proportional gain
   * @param kp
   * @returns true if new value is within [0 1.0]
   */
  inline bool setPrimaryKp(const double kp) {
    bool rtn = true;
    if(kp<= 1.0 && kp>= 0.0){
      kpp_ = kp;
    }
    else{
      rtn = false;
    }
    return(rtn);
  }
  /**
   * @brief Setter for auxillary proportional gain
   * @param kp
   * @returns true if new value is within [0 1.0]
   */
  inline bool setAuxiliaryKp(const double kp) {
    bool rtn = true;
    if(kp<= 1.0 && kp>= 0.0){
      kpa_ = kp;
    }
    else{
      rtn = false;
    }
    return(rtn);
  }
    /**
   * @brief Getter for primary proportional gain
   */
 double getPrimaryKp() const {return kpp_;}
    /**
   * @brief Getter for auxillary proportional gain
   */
 double getAuxillaryKp() const {return kpa_;}

  
 public:

  //TODO document
  virtual Eigen::MatrixXd calcNullspaceProjection(const Eigen::MatrixXd &J) const;

  //TODO document
  virtual Eigen::MatrixXd calcNullspaceProjectionTheRightWay(const Eigen::MatrixXd &A) const;

  //TODO document
  virtual Eigen::MatrixXd calcDampedPseudoinverse(const Eigen::MatrixXd &J) const;

 protected:
  // termination-criteria limits / tolerances
  unsigned int max_iter_;
  double joint_convergence_tol_;
  double kpp_;  /**< primary proportional gain */
  double kpa_; /**< auxillary proportional gain */

  // constraints
  ConstraintGroup primary_constraints_;
  ConstraintGroup auxiliary_constraints_;

  // state/counter data
  bool initialized_;
  SolverState state_;
  basic_kin::BasicKin kin_;

  bool debug_;
  std::vector<Eigen::VectorXd> iteration_path_;

  moveit::core::RobotStatePtr robot_state_;

  /**
   * @brief Pure definition for calculating constraint error
   * @param constraint_type Contraint type (primary or auxiliary)
   * @return Error vector (b-input in calcPInv)
   */
  virtual Eigen::VectorXd calcConstraintError(constraint_types::ConstraintType constraint_type);

  /**
   * @brief Pure definition for calculating Jacobian
   * @param constraint_type Contraint type (primary or auxiliary)
   * @return Jacobian matrix (A-input in calcPInv)
   */
  virtual Eigen::MatrixXd calcConstraintJacobian(constraint_types::ConstraintType constraint_type);


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

