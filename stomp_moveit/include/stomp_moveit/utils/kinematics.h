/**
 * @file kinematics.h
 * @brief This defines kinematic related utilities.
 *
 * @author Jorge Nicho
 * @date June 3, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <trac_ik/trac_ik.hpp>
#include <boost/optional.hpp>


namespace stomp_moveit
{
namespace utils
{

/**
 * @namespace stomp_moveit::utils::kinematics
 * @brief Utility functions related to finding Inverse Kinematics solutions
 */
namespace kinematics
{

  const static double EPSILON = 0.011;  /**< @brief Used in dampening the matrix pseudo inverse calculation */
  const static double LAMBDA = 0.01;    /**< @brief Used in dampening the matrix pseudo inverse calculation */

  MOVEIT_CLASS_FORWARD(IKSolver);

  /**
   * @class stomp_moveit::utils::kinematics::IKSolver
   * @brief Wrapper around an IK solver implementation.
   */
  class IKSolver
  {
  public:
    /**
     * @brief Creates an internal IK solver for the specified robot and planning group
     * @param robot_state The current robot state
     * @param group_name  The planning group name
     * @param max_time    Max time allowed to find a solution.
     */
    IKSolver(const moveit::core::RobotState& robot_state,std::string group_name,double max_time = 0.005);
    ~IKSolver();

    /**
     * @brief Should be called whenever the robot's kinematic state has changed
     * @param state The current robot state
     */
    void setKinematicState(const moveit::core::RobotState& state);

    /**
     * @brief Find the joint position that achieves the requested tool pose.
     * @param seed      A joint pose to seed the solver.
     * @param tool_pose The tool pose for which a joint solution must be found. The frame of reference is assumed to be the model root
     * @param solution  The joint values that place the tool in the requested cartesian pose
     * @param tol       The tolerance values for each dimension of position and orientation relative to the tool pose.
     * @return  True if a solution was found, false otherwise.
     */
    bool solve(const Eigen::VectorXd& seed,const Eigen::Affine3d& tool_pose,Eigen::VectorXd& solution,
               Eigen::VectorXd tol = Eigen::VectorXd::Constant(6,0.005)) ;

    /**
     * @brief Find the joint position that achieves the requested tool pose.
     * @param seed      A joint pose to seed the solver.
     * @param tool_pose The tool pose for which a joint solution must be found. The frame of reference is assumed to be the model root
     * @param solution  The joint values that place the tool in the requested cartesian pose
     * @param tol       The tolerance values for each dimension of position and orientation relative to the tool pose.
     * @return  True if a solution was found, false otherwise.
     */
    bool solve(const std::vector<double>& seed, const Eigen::Affine3d& tool_pose,std::vector<double>& solution,
               std::vector<double> tol = std::vector<double>(6,0.005)) ;

    /**
     * @brief Find the joint position that obeys the specified Cartesian constraint.
     * @param seed              A joint pose to seed the solver.
     * @param tool_constraints  The Cartesian constraint info to be used in determining the tool pose and tolerances
     * @param solution          The joint values that comply with the Cartesian constraint.
     * @return  True if a solution was found, false otherwise.
     */
    bool solve(const std::vector<double>& seed, const moveit_msgs::Constraints& tool_constraints,std::vector<double>& solution);

    /**
     * @brief Find the joint position that obeys the specified Cartesian constraint.
     * @param seed              A joint pose to seed the solver.
     * @param tool_constraints  The Cartesian constraint info to be used in determining the tool pose and tolerances
     * @param solution          The joint values that comply with the Cartesian constraint.
     * @return  True if a solution was found, false otherwise.
     */
    bool solve(const Eigen::VectorXd& seed, const moveit_msgs::Constraints& tool_constraints,Eigen::VectorXd& solution);

  protected:

    std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_impl_;
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::string group_name_;
    Eigen::Affine3d tf_base_to_root_;

  };

  /**
   * @brief Checks if the constraint structured contains valid data from which a proper cartesian constraint can
   *        be produced.
   * @param c   The constraint object. It may define position, orientation or both constraint types.
   * @return True when it is Cartesian, false otherwise.
   */
  bool isCartesianConstraints(const moveit_msgs::Constraints& c);

  /**
   * @brief Populates the missing parts of a Cartesian constraints in order to provide a constraint that can be used by the Ik solver.
   * @param c               The constraint object. It may define position, orientation or both constraint types.
   * @param ref_pose        If no orientation or position constraints are given then this pose will be use to fill the missing information.
   * @param default_pos_tol Used when no position tolerance is specified.
   * @param default_rot_tol Used when no rotation tolerance is specified
   * @return  A constraint object
   */
  boost::optional<moveit_msgs::Constraints> curateCartesianConstraints(const moveit_msgs::Constraints& c,const Eigen::Affine3d& ref_pose,
                                                                double default_pos_tol = 0.0005, double default_rot_tol = M_PI);

  /**
   * @brief Extracts the cartesian data from the constraint message
   * @param model         The robot model
   * @param constraints   A moveit_msgs message that encapsulates the cartesian constraints specifications.
   * @param tool_pose     The tool pose as specified in the constraint in the target_frame.
   * @param tolerance     The tolerance values on the tool pose as specified in the constraint message.
   * @param target_frame  The coordinate frame of the tool pose.  If black the model root is used
   * @return True if succeeded, false otherwise.
   */
  bool decodeCartesianConstraint(moveit::core::RobotModelConstPtr model,const moveit_msgs::Constraints& constraints,
                                 Eigen::Affine3d& tool_pose, std::vector<double>& tolerance, std::string target_frame = "");

  /**
   * @brief Extracts the cartesian data from the constraint message
   * @param model         The robot model
   * @param constraints A moveit_msgs message that encapsulates the cartesian constraints specifications.
   * @param tool_pose   The tool pose as specified in the constraint message.
   * @param tolerance   The tolerance values on the tool pose as specified in the constraint message.
   * @param target_frame  The coordinate frame of the tool pose.  If black the model root is used
   * @return  True if succeeded, false otherwise.
   */
  bool decodeCartesianConstraint(moveit::core::RobotModelConstPtr model,const moveit_msgs::Constraints& constraints,
                                 Eigen::Affine3d& tool_pose, Eigen::VectorXd& tolerance,std::string target_frame = "");

  /**
   * @brief Creates cartesian poses in accordance to the constraint and sampling resolution values
   * @param c                     The constraints which provides information on how to sample the poses.
   * @param sampling_resolution   The incremental values used to sample along each dimension of position and orientation (dx, dy, dz, d_rx, d_ry, d_rz)
   * @param max_samples           Maximum number of samples to be generated
   * @return The sampled poses
   */
  std::vector<Eigen::Affine3d> sampleCartesianPoses(const moveit_msgs::Constraints& c,
                                                    const std::vector<double> sampling_resolution = {0.05, 0.05, 0.05, M_PI_2,M_PI_2,M_PI_2},
                                                    int max_samples =20 );


/**
 * @brief Computes the twist vector [vx vy vz wx wy wz]'  relative to the current tool coordinate system.  The rotational part is
 *        composed of the product between the angle times the axis about which it rotates.
 *
 * @param p0        start tool pose in world coordinates
 * @param pf        final tool pose in world coordinates
 * @param nullity   array of 0's and 1's indicating which cartesian DOF's are unconstrained (0)
 * @param twist     the twist vector in tool coordinates (change from p0 to pf) [6 x 1].
 */
  void computeTwist(const Eigen::Affine3d& p0,
                                          const Eigen::Affine3d& pf,
                                          const Eigen::ArrayXi& nullity,Eigen::VectorXd& twist);

  /**
   * @brief Convenience function to remove entire rows of the Jacobian matrix.
   * @param jacb          The jacobian matrix of size [num_dimensions x 6]
   * @param indices       An indices vector where each entry indicates an row index of the jacobian that will be kept.
   * @param jacb_reduced  The reduced jacobian containing the only the rows indicated by the 'indices' array.
   */
  void reduceJacobian(const Eigen::MatrixXd& jacb,
                                            const std::vector<int>& indices,Eigen::MatrixXd& jacb_reduced);

  /**
   * @brief Calculates the damped pseudo inverse of a matrix using singular value decomposition
   * @param jacb              The jacobian matrix
   * @param jacb_pseudo_inv   The pseudo inverse of the matrix
   * @param eps               Used to threshold the singular values
   * @param lambda            Used in preventing division by small singular values from generating large numbers.
   */
  void calculateDampedPseudoInverse(const Eigen::MatrixXd &jacb, Eigen::MatrixXd &jacb_pseudo_inv,
                                           double eps = 0.011, double lambda = 0.01);

  /**
   * @brief Convenience function to calculate the Jacobian's null space matrix for an under constrained tool cartesian pose.
   * @param state             A pointer to the robot state.
   * @param group             The name of the kinematic group.
   * @param tool_link         The tool link name
   * @param constrained_dofs  A vector of the form [x y z rx ry rz] filled with 0's and 1's to indicate an unconstrained or fully constrained DOF.
   * @param joint_pose        The joint pose at which to compute the jacobian matrix.
   * @param jacb_nullspace    The jacobian null space matrix [num_dimensions x num_dimensions]
   * @return True if a solution was found, false otherwise.
   */
  bool computeJacobianNullSpace(moveit::core::RobotStatePtr state,std::string group,std::string tool_link,
                                       const Eigen::ArrayXi& constrained_dofs,const Eigen::VectorXd& joint_pose,
                                       Eigen::MatrixXd& jacb_nullspace);

} // kinematics
} // utils
} // stomp_moveit



#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UTILS_KINEMATICS_H_ */
