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
#include <moveit/collision_detection_fcl/collision_common.h>
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
  kpp_ = 0.1;// default primary proportional gain
  kpa_ = 0.5;// default auxillary proportional gain
}

Eigen::VectorXd Constrained_IK::calcConstraintError(constraint_types::ConstraintType constraint_type)
{
  switch(constraint_type)
  {
    case constraint_types::primary:
      return primary_constraints_.calcError();
    case constraint_types::auxiliary:
      return auxiliary_constraints_.calcError();
  }
}

Eigen::MatrixXd Constrained_IK::calcConstraintJacobian(constraint_types::ConstraintType constraint_type)
{
  switch(constraint_type)
  {
    case constraint_types::primary:
      return primary_constraints_.calcJacobian();
    case constraint_types::auxiliary:
      return auxiliary_constraints_.calcJacobian();
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


bool Constrained_IK::getDistanceInfo(const std::string link_name, Constrained_IK::DistanceInfo &dist_info) const
{
  std::map<std::string, fcl::DistanceResult>::const_iterator it;
  it = distance_detailed_.find(link_name);

  if (it != distance_detailed_.end())
  {
    fcl::DistanceResult dist = static_cast<const fcl::DistanceResult>(it->second);
    const collision_detection::CollisionGeometryData* cd1 = static_cast<const collision_detection::CollisionGeometryData*>(dist.o1->getUserData());
    const collision_detection::CollisionGeometryData* cd2 = static_cast<const collision_detection::CollisionGeometryData*>(dist.o2->getUserData());
    if (cd1->ptr.link->getName() == link_name)
    {
      dist_info.nearest_obsticle = cd2->ptr.link->getName();
      dist_info.link_point = Eigen::Vector3d(dist.nearest_points[0].data.vs);
      dist_info.obsticle_point = Eigen::Vector3d(dist.nearest_points[1].data.vs);
      dist_info.avoidance_vector = Eigen::Vector3d((dist.nearest_points[1]-dist.nearest_points[0]).data.vs);
      dist_info.avoidance_vector.norm();
      dist_info.distance = dist.min_distance;
    }
    else if (cd2->ptr.link->getName() == link_name)
    {
      dist_info.nearest_obsticle = cd1->ptr.link->getName();
      dist_info.link_point = Eigen::Vector3d(dist.nearest_points[1].data.vs);
      dist_info.obsticle_point = Eigen::Vector3d(dist.nearest_points[0].data.vs);
      dist_info.avoidance_vector = Eigen::Vector3d((dist.nearest_points[0]-dist.nearest_points[1]).data.vs);
      dist_info.avoidance_vector.norm();
      dist_info.distance = dist.min_distance;
    }
    else
    {
      ROS_ERROR("getDistanceInfo was unable to find link after match!");
      return false;
    }
    return true;
  }
  else
  {
    ROS_ERROR("couldn't find link with that name %s", link_name.c_str());
    for( it=distance_detailed_.begin(); it != distance_detailed_.end(); it++)
    {
      ROS_ERROR("name: %s", it->first.c_str());
    }
    return false;
  }
}

void Constrained_IK::calcInvKin(const Eigen::Affine3d &goal,
                                const Eigen::VectorXd &joint_seed,
                                Eigen::VectorXd &joint_angles,
                                int min_updates) 
{
  calcInvKin(goal,joint_seed, planning_scene::PlanningSceneConstPtr(), joint_angles, min_updates);
}

void Constrained_IK::calcInvKin(const Eigen::Affine3d &goal,
                                const Eigen::VectorXd &joint_seed,
                                const planning_scene::PlanningSceneConstPtr planning_scene,
                                Eigen::VectorXd &joint_angles,
                                int min_updates) 
{
  if (!checkInitialized(constraint_types::primary))
    throw std::runtime_error("Must call init() before using Constrained_IK");
  //TODO should goal be checked here instead of in reset()?

  // initialize state
  joint_angles = joint_seed;  // initialize result to seed value
  reset(goal, joint_seed);    // reset state vars for this IK solve
  update(joint_angles);       // update current state

  if(planning_scene && collision_checks_required())// initialize detailed distance_ prior to checking statusn
  {
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene->getCurrentState()));
    robot_state->setJointGroupPositions(kin_.getJointModelGroup()->getName(),joint_angles);
    robot_state->update();
    distance_detailed_ = planning_scene->getCollisionRobot()->distanceSelfDetailed(*robot_state, planning_scene->getAllowedCollisionMatrix());
  }

  // iterate until solution converges (or aborted)
  while (!checkStatus() || min_updates>0)
  {
    // If planning_scene is not null we calculate distance data for collision
    // avoidance.
    if(planning_scene && collision_checks_required())
    {
      moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene->getCurrentState()));
      robot_state->setJointGroupPositions(kin_.getJointModelGroup()->getName(),joint_angles);
      robot_state->update();
      distance_detailed_ = planning_scene->getCollisionRobot()->distanceSelfDetailed(*robot_state, planning_scene->getAllowedCollisionMatrix());
    }
    // calculate a Jacobian (relating joint-space updates/deltas to cartesian-space errors/deltas)
    // and the associated cartesian-space error/delta vector
    // Primary Constraints

    MatrixXd J_p  = calcConstraintJacobian(constraint_types::primary);
    // TODO since we already have J_p = USV, use that to get null-projection too.
    // otherwise, we are repeating the expensive calculation of the SVD
    MatrixXd Ji_p = calcDampedPseudoinverse(J_p);
    int rows = J_p.rows();
    int cols = J_p.cols();
    VectorXd err_p = calcConstraintError(constraint_types::primary);
    // solve for the resulting joint-space update
    VectorXd dJoint_p;
    VectorXd dJoint_a;

    dJoint_p = kpp_*(Ji_p*err_p);
    if(dJoint_p.norm() > 1.0)// limit maximum update to unit size 1 radian /meter
    {
      dJoint_p = dJoint_p/dJoint_p.norm();
    }
    ROS_DEBUG("theta_p = %f ep = %f",dJoint_p.norm(), err_p.norm());

    // Auxiliary Constraints
    dJoint_a.setZero(dJoint_p.size());
    if (checkInitialized(constraint_types::auxiliary)) 
    {
      MatrixXd J_a  = calcConstraintJacobian(constraint_types::auxiliary);
      VectorXd err_a = calcConstraintError(constraint_types::auxiliary);
      MatrixXd N_p = calcNullspaceProjectionTheRightWay(J_p);
      MatrixXd Jnull_a = calcDampedPseudoinverse(J_a*N_p);
      dJoint_a = kpa_*Jnull_a*(err_a-J_a*dJoint_p);
      if(state_.iter > max_iter_*.9) // print debugging info if not converging 
      {
        ROS_ERROR("theta_p = %f theta_a = %f ep = %f ea = %f", dJoint_p.norm(), dJoint_a.norm(), err_p.norm(), err_a.norm());
      }
    }

    if(state_.iter > max_iter_*.9) // print debugging info if not converging 
    {
      if(!checkInitialized(constraint_types::auxiliary))
        ROS_ERROR("theta_p = %f ep = %f ", dJoint_p.norm(),  err_p.norm());
    }

    // update joint solution by the calculated update (or a partial fraction)
    joint_angles += (dJoint_p + dJoint_a);
    clipToJointLimits(joint_angles);

    // re-update internal state variables
    update(joint_angles);
    min_updates--;
  }

  // checking for collision on a valid planning scene
  if(planning_scene && collision_checks_required())
  {
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene->getCurrentState()));
    robot_state->setJointGroupPositions(kin_.getJointModelGroup()->getName(),joint_angles);
    robot_state->update();
    if(planning_scene->isStateColliding(*robot_state,kin_.getJointModelGroup()->getName()))
    {
      ROS_ERROR("Robot is in collision at this pose");
    }
  }

  ROS_INFO_STREAM("IK solution: " << joint_angles.transpose());
}

bool Constrained_IK::checkStatus() const
{
  // check constraints for completion
  bool primary_status = primary_constraints_.checkStatus();
  bool auxiliary_status = auxiliary_constraints_.checkStatus();

  if (primary_status && auxiliary_status)
    return true;

  // check maximum iterations
  if (state_.iter > max_iter_)
    throw std::runtime_error("Maximum iterations reached.  IK solution may be invalid.");

  // check for joint convergence
  //   - this is an error: joints stabilize, but goal pose not reached
  if (state_.joints_delta.cwiseAbs().maxCoeff() < joint_convergence_tol_)
  {
//	  ROS_ERROR_STREAM("Reached " << state_.iter << " / " << max_iter_ << " iterations before convergence.");
//	  throw std::runtime_error("Iteration converged before goal reached.  IK solution may be invalid");

	  ROS_WARN_STREAM("Reached " << state_.iter << " / " << max_iter_ << " iterations before convergence.");
      return true;
  }

  return false;
}

void Constrained_IK::clearConstraintList()
{
  primary_constraints_.clear();
  auxiliary_constraints_.clear();
}

void Constrained_IK::clipToJointLimits(Eigen::VectorXd &joints)
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

void Constrained_IK::reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed)
{
  if (!kin_.checkJoints(joint_seed))
    throw std::invalid_argument("Seed doesn't match kinematic model");

  if (!goal.matrix().block(0,0,3,3).isUnitary(1e-6))
        throw std::invalid_argument("Goal pose not proper affine");

  state_.reset(goal, joint_seed);  // reset state
  primary_constraints_.reset();            // reset primary constraints
  auxiliary_constraints_.reset();            // reset auxiliary constraints
}

void Constrained_IK::update(const Eigen::VectorXd &joints)
{
  // update maximum iterations
  state_.iter++;

  // update joint convergence
  state_.joints_delta = joints - state_.joints;
  state_.joints = joints;
  kin_.calcFwdKin(joints, state_.pose_estimate);

  if (debug_)
      iteration_path_.push_back(joints);

  primary_constraints_.update(state_);
  auxiliary_constraints_.update(state_);
}

bool Constrained_IK::collision_checks_required()
{
  return( primary_constraints_.collision_checks_required() | auxiliary_constraints_.collision_checks_required());
}


} // namespace constrained_ik

