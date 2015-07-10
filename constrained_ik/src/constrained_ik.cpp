/**
 * @file constrained_ik.cpp
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
#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraint_group.h"
#include <boost/make_shared.hpp>
#include <constrained_ik/collision_robot_fcl_detailed.h>
#include <constrained_ik/constraint_results.h>
#include <ros/ros.h>

namespace constrained_ik
{
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Affine3d;

Constrained_IK::Constrained_IK()
{
  initialized_ = false;
  max_iter_ = 500;                  //default max_iter
  joint_convergence_tol_ = 0.0001;   //default convergence tolerance
  debug_ = false;
  kpp_ = 1.0;// default primary proportional gain
  kpa_ = 0.5;// default auxillary proportional gain
}

constrained_ik::ConstraintResults Constrained_IK::evalConstraint(constraint_types::ConstraintTypes constraint_type, const constrained_ik::SolverState &state) const
{
  switch(constraint_type)
  {
    case constraint_types::Primary:
      return primary_constraints_.evalConstraint(state);
    case constraint_types::Auxiliary:
      return auxiliary_constraints_.evalConstraint(state);
  }
}

Eigen::MatrixXd Constrained_IK::calcNullspaceProjection(const Eigen::MatrixXd &J) const
{
  MatrixXd J_pinv = calcDampedPseudoinverse(J);
  MatrixXd JplusJ = J_pinv * J;
  int mn = JplusJ.rows();
  MatrixXd P = MatrixXd::Identity(mn,mn)-JplusJ;
  return (P);
}

Eigen::MatrixXd Constrained_IK::calcNullspaceProjectionTheRightWay(const Eigen::MatrixXd &A) const
{
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullV);
  MatrixXd V(svd.matrixV());

  // determine rank using same algorithym as latest Eigen
  // TODO replace next 10 lines of code with rnk = svd.rank(); once eigen3 is updated
  int rnk = 0;
  if(svd.singularValues().size()==0)
  {
    rnk = 0;
  }
  else
  {
    double threshold = std::min(A.rows(),A.cols())*Eigen::NumTraits<double>::epsilon();
    double premultiplied_threshold = svd.singularValues().coeff(0) * threshold;
    rnk = svd.nonzeroSingularValues()-1;
    while(rnk>=0 && svd.singularValues().coeff(rnk) < premultiplied_threshold) --rnk;
    rnk++;
  }

  // zero singular vectors in the range of A
  for(int i=0; i<rnk; ++i)
  {
    for(int j=0; j<A.cols(); j++) V(j,i) = 0;
  }

  MatrixXd P = V *  V.transpose();
  return(P);
}

Eigen::MatrixXd Constrained_IK::calcDampedPseudoinverse(const Eigen::MatrixXd &J) const
{
  MatrixXd J_pinv;

  if (basic_kin::BasicKin::dampedPInv(J,J_pinv))
  {
    return J_pinv;
  }
  else
  {
    ROS_ERROR_STREAM("Not able to calculate damped pseudoinverse!");
    throw std::runtime_error("Not able to calculate damped pseudoinverse!  IK solution may be invalid.");
  }
}

void Constrained_IK::calcInvKin(const Eigen::Affine3d &goal,
                                const Eigen::VectorXd &joint_seed,
                                Eigen::VectorXd &joint_angles,
                                int min_updates) const
{
  calcInvKin(goal,joint_seed, planning_scene::PlanningSceneConstPtr(), joint_angles, min_updates);
}

void Constrained_IK::calcInvKin(const Eigen::Affine3d &goal,
                                const Eigen::VectorXd &joint_seed,
                                const planning_scene::PlanningSceneConstPtr planning_scene,
                                Eigen::VectorXd &joint_angles,
                                int min_updates) const
{
    // initialize state
  joint_angles = joint_seed;  // initialize result to seed value
  constrained_ik::SolverState state = getState(goal, joint_seed); // create state vars for this IK solve
  state.condition = checkInitialized();
  state.planning_scene = planning_scene;
  if(planning_scene)
  {
    state.robot_state = robot_state::RobotStatePtr(new moveit::core::RobotState(planning_scene->getCurrentState()));
    state.collision_robot = CollisionRobotFCLDetailed::CollisionRobotFCLDetailedPtr(new CollisionRobotFCLDetailed(planning_scene->getRobotModel()));
  }

  if (state.condition == initialization_state::NothingInitialized || state.condition == initialization_state::AuxiliaryOnly)
    throw std::runtime_error("Must call init() before using Constrained_IK and have a primary constraint.");

  //Cache the joint angles to return if max iteration is reached.
  Eigen::VectorXd cached_joint_angles = joint_seed;

  // iterate until solution converges (or aborted)
  bool status = false;
  while (true)
  {
    // re-update internal state variables
    updateState(state, joint_angles);

    // calculate a Jacobian (relating joint-space updates/deltas to cartesian-space errors/deltas)
    // and the associated cartesian-space error/delta vector
    // Primary Constraints
    constrained_ik::ConstraintResults primary = evalConstraint(constraint_types::Primary, state);
    // TODO since we already have J_p = USV, use that to get null-projection too.
    // otherwise, we are repeating the expensive calculation of the SVD
    MatrixXd Ji_p = calcDampedPseudoinverse(primary.jacobian);
    VectorXd dJoint_p = kpp_*(Ji_p*primary.error);
    if(dJoint_p.norm() > 1.0)// limit maximum update to unit size 1 radian /meter
    {
      dJoint_p = dJoint_p/dJoint_p.norm();
    }
    ROS_DEBUG("theta_p = %f ep = %f",dJoint_p.norm(), primary.error.norm());


    // Auxiliary Constraints
    VectorXd dJoint_a;
    constrained_ik::ConstraintResults auxiliary;
    dJoint_a.setZero(dJoint_p.size());
    if (state.condition == initialization_state::PrimaryAndAuxiliary)
    {
      auxiliary = evalConstraint(constraint_types::Auxiliary, state);
      if (!auxiliary.isEmpty()) // This is required because not all constraints always return data.
      {
        MatrixXd N_p = calcNullspaceProjectionTheRightWay(auxiliary.jacobian);
        MatrixXd Jnull_a = calcDampedPseudoinverse(auxiliary.jacobian*N_p);
        dJoint_a = kpa_*Jnull_a*(auxiliary.error-auxiliary.jacobian*dJoint_p);
      }
    }

    // Check the status of convergence
    if(state.condition == initialization_state::PrimaryAndAuxiliary)
    {
      status = checkStatus(state, primary, auxiliary);

      if(state.iter > max_iter_*.9)
        ROS_ERROR("theta_p = %f theta_a = %f ep = %f ea = %f", dJoint_p.norm(), dJoint_a.norm(), primary.error.norm(), auxiliary.error.norm());
    }
    else if(state.condition == initialization_state::PrimaryOnly)
    {
      status = checkStatus(state, primary);

      if(state.iter > max_iter_*.9)
        ROS_ERROR("theta_p = %f ep = %f ", dJoint_p.norm(),  primary.error.norm());
    }

    if(status && min_updates<=0)
    {
      break;
    }
    else
    {
      // update joint solution by the calculated update (or a partial fraction)
      joint_angles += (dJoint_p + dJoint_a);
      clipToJointLimits(joint_angles);
      min_updates--;
    }
  }

  if (state.iter == max_iter_)
    joint_angles = cached_joint_angles;

  // checking for collision on a valid planning scene
  if(state.planning_scene)
  {
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(state.planning_scene->getCurrentState()));
    robot_state->setJointGroupPositions(kin_.getJointModelGroup()->getName(),joint_angles);
    robot_state->update();
    if(state.planning_scene->isStateColliding(*robot_state,kin_.getJointModelGroup()->getName()))
    {
      ROS_ERROR("Robot is in collision at this pose");
    }
  }

  ROS_DEBUG_STREAM("IK solution: " << joint_angles.transpose());
}

bool Constrained_IK::checkStatus(const constrained_ik::SolverState &state, const constrained_ik::ConstraintResults &primary) const
{
  if (primary.status)
    return true;

  // check maximum iterations
  if (state.iter > max_iter_)
    throw std::runtime_error("Maximum iterations reached.  IK solution may be invalid.");

  // check for joint convergence
  //   - this is an error: joints stabilize, but goal pose not reached
  if (state.joints_delta.cwiseAbs().maxCoeff() < joint_convergence_tol_)
  {
    ROS_WARN_STREAM("Reached " << state.iter << " / " << max_iter_ << " iterations before convergence.");
    return true;
  }

  return false;
}

bool Constrained_IK::checkStatus(const constrained_ik::SolverState &state, const constrained_ik::ConstraintResults &primary, const constrained_ik::ConstraintResults &auxiliary) const
{
  if (primary.status && auxiliary.status)
    return true;

  // check maximum iterations
  if (state.iter > max_iter_)
    throw std::runtime_error("Maximum iterations reached.  IK solution may be invalid.");

  // check for joint convergence
  //   - this is an error: joints stabilize, but goal pose not reached
  if (state.joints_delta.cwiseAbs().maxCoeff() < joint_convergence_tol_)
  {
    ROS_WARN_STREAM("Reached " << state.iter << " / " << max_iter_ << " iterations before convergence.");
    return true;
  }

  return false;
}

void Constrained_IK::clearConstraintList()
{
  primary_constraints_.clear();
  auxiliary_constraints_.clear();
}

void Constrained_IK::clipToJointLimits(Eigen::VectorXd &joints) const
{
  const MatrixXd limits = kin_.getLimits();
  const VectorXd orig_joints(joints);

  if (joints.size() != limits.rows())
    throw std::invalid_argument("clipToJointLimits: Unexpected number of joints");

  for (size_t i=0; i<limits.rows(); ++i)
  {
    joints[i] = std::max(limits(i,0), std::min(limits(i,1), joints[i]));
  }
  if (debug_ && !joints.isApprox(orig_joints))
      ROS_WARN("Joints have been clipped");
}

void Constrained_IK::init(const basic_kin::BasicKin &kin)
{
  if (!kin.checkInitialized())
    throw std::invalid_argument("Input argument 'BasicKin' must be initialized");

  ros::NodeHandle pnh("~");
  pnh.getParam("primary_kp", kpp_);
  pnh.getParam("auxiliary_kp", kpa_);
  ROS_INFO("using primary kp=%f and auxiliary kp=%f", kpp_, kpa_);
  kin_ = kin;
  initialized_ = true;
  primary_constraints_.init(this);
  auxiliary_constraints_.init(this);

}

double Constrained_IK::rangedAngle(double angle)
{
    angle = copysign(fmod(fabs(angle),2.0*M_PI), angle);
    if (angle < -M_PI) return angle+2.*M_PI;
    if (angle > M_PI)  return angle-2.*M_PI;
    return angle;
}

constrained_ik::SolverState Constrained_IK::getState(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed) const
{
  if (!kin_.checkJoints(joint_seed))
    throw std::invalid_argument("Seed doesn't match kinematic model");

  if (!goal.matrix().block(0,0,3,3).isUnitary(1e-6))
        throw std::invalid_argument("Goal pose not proper affine");

  return constrained_ik::SolverState(goal, joint_seed);
}

void Constrained_IK::updateState(constrained_ik::SolverState &state, const Eigen::VectorXd &joints) const
{
  // update maximum iterations
  state.iter++;

  // update joint convergence
  state.joints_delta = joints - state.joints;
  state.joints = joints;
  kin_.calcFwdKin(joints, state.pose_estimate);

  if(state.planning_scene && state.robot_state)
  {
    state.robot_state->setJointGroupPositions(kin_.getJointModelGroup()->getName(), joints);
    state.robot_state->update();
  }

  if (debug_)
      state.iteration_path.push_back(joints);

}

} // namespace constrained_ik

