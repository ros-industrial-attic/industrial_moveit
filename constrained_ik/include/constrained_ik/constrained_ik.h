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
#include <constrained_ik/constraint_results.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <constrained_ik/ConstrainedIKDynamicReconfigureConfig.h>

namespace constrained_ik
{

/**
 * @brief Damped Least-Squares Inverse Kinematic Solution
 *          - see derived classes for more complex constrained-IK solvers
 */
class Constrained_IK
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  virtual void addConstraint(Constraint* constraint, ConstraintTypes constraint_type)
  {
    switch(constraint_type)
    {
      case constraint_types::Primary:
        primary_constraints_.add(constraint);
        break;
      case constraint_types::Auxiliary:
        auxiliary_constraints_.add(constraint);
        break;
    }
  }

  /**
   * @brief computes the inverse kinematics for the given pose of the tip link
   * @param pose  The pose of the tip link
   * @param joint_seed joint values that is used as the initial guess
   * @param joint_angles The joint pose that places the tip link to the desired pose.
   */
  virtual void calcInvKin(const Eigen::Affine3d &goal,
                          const Eigen::VectorXd &joint_seed,
                          Eigen::VectorXd &joint_angles) const;


  /**
   * @brief computes the inverse kinematics for the given pose of the tip link
   * @param pose  The pose of the tip link
   * @param joint_seed joint values that is used as the initial guess
   * @param planning_scene pointer to a planning scene that holds all the object in the environment.  Use by the solver to check for collision; if
   *            a null pointer is passed then collisions are ignored.
   * @param joint_angles The joint pose that places the tip link to the desired pose.
   */
  virtual void calcInvKin(const Eigen::Affine3d &goal,
                          const Eigen::VectorXd &joint_seed,
                          const planning_scene::PlanningSceneConstPtr planning_scene,
                          Eigen::VectorXd &joint_angles) const;

  /**
   * @brief Checks to see if object is initialized (ie: init() has been called)
   * @return InitializationState
   */
  initialization_state::InitializationState checkInitialized() const
  {
    if (initialized_)
    {
      if (!primary_constraints_.empty() && !auxiliary_constraints_.empty())
      {
        return initialization_state::PrimaryAndAuxiliary;
      }
      else if (!primary_constraints_.empty() && auxiliary_constraints_.empty())
      {
        return initialization_state::PrimaryOnly;
      }
      else if (primary_constraints_.empty() && !auxiliary_constraints_.empty())
      {
        return initialization_state::AuxiliaryOnly;
      }
    }
    else
    {
      return initialization_state::NothingInitialized;
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
   * @brief Getter for solver configuration
   * @return ConstrainedIKDynamicReconfigureConfig
   */
  ConstrainedIKDynamicReconfigureConfig getSolverConfiguration() const {return config_;}

  /**
   * @brief Setter for solver configuration
   * @param config new object for config_
   */
  void setSolverConfiguration(const ConstrainedIKDynamicReconfigureConfig &config) {config_ = config;}

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

  //TODO document
  virtual Eigen::MatrixXd calcNullspaceProjection(const Eigen::MatrixXd &J) const;

  //TODO document
  virtual Eigen::MatrixXd calcNullspaceProjectionTheRightWay(const Eigen::MatrixXd &A) const;

  //TODO document
  virtual Eigen::MatrixXd calcDampedPseudoinverse(const Eigen::MatrixXd &J) const;

  void dynamicReconfigureCallback(ConstrainedIKDynamicReconfigureConfig &config, uint32_t level);

 protected:
  // solver configuration parameters
  ros::NodeHandle nh_;
  ConstrainedIKDynamicReconfigureConfig config_;
  boost::scoped_ptr<dynamic_reconfigure::Server<ConstrainedIKDynamicReconfigureConfig> > dynamic_reconfigure_server_;
  boost::recursive_mutex mutex_;

  // constraints
  ConstraintGroup primary_constraints_;
  ConstraintGroup auxiliary_constraints_;

  // state/counter data
  bool initialized_;
  basic_kin::BasicKin kin_;

  bool debug_;

  /**
   * @brief Calculating error, jacobian & status for all constraints specified.
   * @param constraint_type Contraint type (primary or auxiliary)
   * @param state The state of the current solver
   * @return ConstraintResults
   */
  constrained_ik::ConstraintResults evalConstraint(constraint_types::ConstraintTypes constraint_type, const constrained_ik::SolverState &state) const;

  /**
   * @brief This function clips the joints within the joint limits.
   * @param joints a Eigen::VectorXd passed by reference
   */
  void clipToJointLimits(Eigen::VectorXd &joints) const;

  /**
   * @brief Method determine convergence when both primary
   * and auxiliary constraints are present
   * @param state, The state of the current solver
   * @param primary, The primary constraint results
   * @param auxiliary, The auxiliary constraint results
   * @return bool, True for converged, False for not converged
   */
  virtual bool checkStatus(const constrained_ik::SolverState &state, const constrained_ik::ConstraintResults &primary, const constrained_ik::ConstraintResults &auxiliary) const;

  /**
   * @brief Creates a new SolverState and checks key elements.
   * @param goal, The goal of the solver
   * @param joint_seed, The inital joint position for the solver.
   * @return SolverState
   */
  virtual constrained_ik::SolverState getState(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed) const;

  /**
   * @brief Method update an existing SolverState provided
   * new joint positions.
   * @param state, The state of the current solver
   * @param joints, The new joint position to be used by the solver
   */
  virtual void updateState(constrained_ik::SolverState &state, const Eigen::VectorXd &joints) const;

}; // class Constrained_IK

} // namespace constrained_ik

#endif // CONSTRAINED_IK_H

