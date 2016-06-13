/*
 * kinematics.h
 *
 *  Created on: Jun 3, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UTILS_KINEMATICS_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UTILS_KINEMATICS_H_

#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <XmlRpcException.h>



namespace stomp_moveit
{
namespace utils
{
namespace kinematics
{

  const static double EPSILON = 0.1;
  const static double LAMBDA = 0.01;


  void computeTwist(const Eigen::Affine3d& p0,
                                          const Eigen::Affine3d& pf,
                                          const Eigen::ArrayXi& nullity,Eigen::VectorXd& twist)
  {
    twist.resize(nullity.size());
    twist.setConstant(0);
    Eigen::Vector3d twist_pos = pf.translation() - p0.translation();

    // relative rotation -> R = inverse(R0) * Rf
    Eigen::AngleAxisd relative_rot(p0.rotation().transpose() * pf.rotation());
    double angle = relative_rot.angle();
    Eigen::Vector3d axis = relative_rot.axis();

    // forcing angle to range [-pi , pi]
    while( (angle > M_PI) || (angle < -M_PI))
    {
      angle = (angle >  M_PI) ? (angle - 2*M_PI) : angle;
      angle = (angle < -M_PI )? (angle + 2*M_PI) : angle;
    }

    // creating twist rotation relative to tool
    Eigen::Vector3d twist_rot = axis * angle;

    // assigning into full 6dof twist vector
    twist.head(3) = twist_pos;
    twist.tail(3) = twist_rot;

    // zeroing all underconstrained cartesian dofs
    twist = (nullity == 0).select(0,twist);
  }

  void reduceJacobian(const Eigen::MatrixXd& jacb,
                                            const std::vector<int>& indices,Eigen::MatrixXd& jacb_reduced)
  {
    jacb_reduced.resize(indices.size(),jacb.cols());
    for(auto i = 0u; i < indices.size(); i++)
    {
      jacb_reduced.row(i) = jacb.row(indices[i]);
    }
  }

  static void calculateDampedPseudoInverse(const Eigen::MatrixXd &jacb, Eigen::MatrixXd &jacb_pseudo_inv,
                                           double eps = 0.011, double lambda = 0.01)
  {
    using namespace Eigen;


    //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
    //in order to solve Ax=b -> x*=A+ b
    Eigen::JacobiSVD<MatrixXd> svd(jacb, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const MatrixXd &U = svd.matrixU();
    const VectorXd &Sv = svd.singularValues();
    const MatrixXd &V = svd.matrixV();

    // calculate the reciprocal of Singular-Values
    // damp inverse with lambda so that inverse doesn't oscillate near solution
    size_t nSv = Sv.size();
    VectorXd inv_Sv(nSv);
    for(size_t i=0; i< nSv; ++i)
    {
      if (fabs(Sv(i)) > eps)
      {
        inv_Sv(i) = 1/Sv(i);
      }
      else
      {
        inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
      }
    }

    jacb_pseudo_inv = V * inv_Sv.asDiagonal() * U.transpose();
  }

  bool solveIK(moveit::core::RobotStatePtr robot_state, const std::string& group_name, const Eigen::ArrayXi& constrained_dofs,
               const Eigen::ArrayXd& joint_update_rates,const Eigen::ArrayXd& cartesian_convergence_thresholds, int max_iterations,
               const Eigen::Affine3d& tool_goal_pose,const Eigen::VectorXd& init_joint_pose,
               Eigen::VectorXd& joint_pose)
  {
    using namespace Eigen;
    using namespace moveit::core;

    // joint variables
    VectorXd delta_j = VectorXd::Zero(init_joint_pose.size());
    joint_pose = init_joint_pose;
    const JointModelGroup* joint_group = robot_state->getJointModelGroup(group_name);
    robot_state->setJointGroupPositions(joint_group,joint_pose);
    std::string tool_link = joint_group->getLinkModelNames().back();
    Affine3d tool_current_pose = robot_state->getGlobalLinkTransform(tool_link);

    // tool twist variables
    VectorXd tool_twist, tool_twist_reduced;
    std::vector<int> indices;
    for(auto i = 0u; i < constrained_dofs.size(); i++)
    {
      if(constrained_dofs(i) != 0)
      {
        indices.push_back(i);
      }
    }
    tool_twist_reduced = VectorXd::Zero(indices.size());

    // jacobian calculation variables
    MatrixXd identity = MatrixXd::Identity(init_joint_pose.size(),init_joint_pose.size());
    MatrixXd jacb, jacb_reduced, jacb_pseudo_inv;

    unsigned int iteration_count = 0;
    bool converged = false;
    while(iteration_count < max_iterations)
    {

      // computing twist vector
      computeTwist(tool_current_pose,tool_goal_pose,constrained_dofs,tool_twist);

      // check convergence
      if((tool_twist.cwiseAbs().array() < cartesian_convergence_thresholds).all())
      {
        // converged
        converged = true;
        //ROS_DEBUG("Found numeric ik solution after %i iterations",iteration_count);
        break;
      }

      // updating reduced tool twist
      for(auto i = 0u; i < indices.size(); i++)
      {
        tool_twist_reduced(i) = tool_twist(indices[i]);
      }

      // computing jacobian
      if(!robot_state->getJacobian(joint_group,robot_state->getLinkModel(tool_link),Vector3d::Zero(),jacb))
      {
        ROS_ERROR("Failed to get Jacobian for link %s",tool_link.c_str());
        return false;
      }

      // transform jacobian rotational part to tool coordinates
      jacb.bottomRows(3) = tool_current_pose.rotation().transpose()*jacb.bottomRows(3);

      // reduce jacobian and compute its pseudo inverse
      reduceJacobian(jacb,indices,jacb_reduced);
      calculateDampedPseudoInverse(jacb_reduced,jacb_pseudo_inv,EPSILON,LAMBDA);

      // computing joint change
      delta_j = (jacb_pseudo_inv*tool_twist_reduced);

      // updating joint values
      joint_pose += (joint_update_rates* delta_j.array()).matrix();

      // updating tool pose
      robot_state->setJointGroupPositions(joint_group,joint_pose);
      tool_current_pose = robot_state->getGlobalLinkTransform(tool_link);

      iteration_count++;
    }

    //ROS_DEBUG_STREAM("Final tool twist "<<tool_twist.transpose());

    return converged;
  }


} // kinematics
} // utils
} // stomp_moveit



#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UTILS_KINEMATICS_H_ */
