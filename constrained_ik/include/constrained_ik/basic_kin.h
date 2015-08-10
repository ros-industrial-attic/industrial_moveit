/**
 * @file basic_kin.h
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
#ifndef BASIC_KIN_H
#define BASIC_KIN_H

#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <moveit/robot_model/joint_model_group.h>

namespace constrained_ik
{
namespace basic_kin
{

/**
 * @brief Basic low-level kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 */
class BasicKin
{
public:
  BasicKin() :
    initialized_(false),
    group_(NULL)
  {

  }
  ~BasicKin() {}

  /**
   * @brief Calculates tool pose of robot chain
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @param pose Transform of end-of-tip relative to root
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose) const;

  //TODO test
  /**
   * @brief Creates chain and calculates tool pose relative to root
   * New chain is not stored permanently, but subchain_fk_solver_ is updated
   * @param joint_angles Vector of joint angles (size must match number of joints in chain)
   * @param base Name of base link for new chain
   * @param tip Name of tip link for new chain
   * @param pose Transform of end-of-tip relative to base
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcFwdKin(const Eigen::VectorXd &joint_angles,
                  const std::string &base,
                  const std::string &tip,
                  KDL::Frame &pose) const;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param jacobian Output jacobian
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian) const;

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const { return initialized_; }

  //TODO test
  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  bool checkJoints(const Eigen::VectorXd &vec) const;

  //TODO test
  /**
   * @brief Get list of joint names for robot
   * @param names Output vector of joint names, copied from joint_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  bool getJointNames(std::vector<std::string> &names) const;


//   * @brief Get list of joint names for specific robot chain
//   * Crawls chain to create list
//   * @param chain Input robot chain to retrieve list from
//   * @param names Output vector of joint names
//   * @return True if BasicKin has been successfully initialized
//  bool getJointNames(const KDL::Chain &chain, std::vector<std::string> &names) const;

  /**
   * @brief Getter for joint_limits_
   * @return Matrix of joint limits
   */
  Eigen::MatrixXd getLimits() const { return joint_limits_; }

  //TODO test
  /**
   * @brief Get list of link names for robot
   * @param names Output vector of names, copied from link_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  bool getLinkNames(std::vector<std::string> &names) const;

  //TODO test
//   * @brief Get list of link names for specific robot chain
//   * Crawls chain to create list
//   * @param chain Input robot chain to retrieve list from
//   * @param names Output vector of link names
//   * @return True if BasicKin has been successfully initialized
//  bool getLinkNames(const KDL::Chain &chain, std::vector<std::string> &names) const;

  /**
   * @brief Initializes BasicKin
   * Creates KDL::Chain from urdf::Model, populates joint_list_, joint_limits_, and link_list_
   * @param robot Input model containing robot information
   * @param base_name Input name of base link
   * @param tip_name Input name of tip link
   * @param group_name Input name of the kinematic group
   * @return True if init() completes successfully
   */
  bool init(const moveit::core::JointModelGroup* group);

  /**
   * @brief Get the name of the kinematic group
   * @return string with the group name
   */
  const moveit::core::JointModelGroup* getJointModelGroup() const {return group_;}

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  unsigned int numJoints() const { return robot_chain_.getNrOfJoints(); }

  /**
   * @brief get a subchain of the kinematic group
   * @return subchain
   */
  KDL::Chain getSubChain(std::string link_name) const;

  /**
   * @brief Calculates transforms of each link relative to base (not including base)
   * If link_names is specified, only listed links will be returned. Otherwise all links in link_list_ will be returned
   * @param joint_angles Input vector of joint values
   * @param poses Output poses of listed links
   * @param link_names Optional input list of links to calculate transforms for
   * @return True if all requested links have poses calculated
   */
  bool linkTransforms(const Eigen::VectorXd &joint_angles,
                      std::vector<KDL::Frame> &poses,
                      const std::vector<std::string> &link_names = std::vector<std::string>()) const;

  /**
   * @brief Returns the location of the robot base in world coordinate frame.
   * @return Eigen::Affine3d
   */
  Eigen::Affine3d getRobotBaseInWorld() const { return robot_base_pose_; }

  /**
   * @brief Assigns values from another BasicKin to this
   * @param rhs Input BasicKin object to copy from
   * @return reference to this BasicKin object
   */
  BasicKin& operator=(const BasicKin& rhs);

  /**
   * @brief Solve equation Ax=b for x
   * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
   * @param A Input matrix (represents Jacobian)
   * @param b Input vector (represents desired pose)
   * @param x Output vector (represents joint values)
   * @return True if solver completes properly
   */
  bool solvePInv(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x) const;

  /**
   * @brief Calculate Damped Pseudoinverse
   * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
   * @param A Input matrix (represents Jacobian)
   * @param P Output matrix (represents pseudoinverse of A)
   * @return True if Pseudoinverse completes properly
   */
static  bool dampedPInv(const Eigen::MatrixXd &A, Eigen::MatrixXd &P);

private:
  bool initialized_;
  const moveit::core::JointModelGroup* group_;
  KDL::Chain  robot_chain_;
  KDL::Tree   kdl_tree_;
  std::string base_name_,  tip_name_, world_frame_;
  Eigen::Affine3d robot_base_pose_;
  std::vector<std::string> joint_list_, link_list_;
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_limits_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  /**
   * @brief Convert Eigen::Vector to KDL::JntArray
   * @param vec Input Eigen vector
   * @param joints Output KDL joint array
   */
  static void EigenToKDL(const Eigen::VectorXd &vec, KDL::JntArray &joints) {joints.data = vec;}

  /**
   * @brief Get joint number of given joint in initialized robot
   * @param joint_name Input name of joint
   * @return joint index if joint_name part of joint_list_, n+1 otherwise
   */
  int getJointNum(const std::string &joint_name) const;

  /**
   * @brief Get link number of given joint in initialized robot
   * @param link_name Input name of link
   * @return link index if link_name part of link_list_, l+1 otherwise
   */
  int getLinkNum(const std::string &link_name) const;

  /**
   * @brief Convert KDL::Frame to Eigen::Affine3d
   * @param frame Input KDL Frame
   * @param transform Output Eigen transform (Affine3d)
   */
  static void KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform);

  /**
   * @brief Convert KDL::Jacobian to Eigen::Matrix
   * @param jacobian Input KDL Jacobian
   * @param matrix Output Eigen MatrixXd
   */
  static void KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class BasicKin

} // namespace basic_kin
} // namespace constrained_ik


#endif // BASIC_KIN_H

