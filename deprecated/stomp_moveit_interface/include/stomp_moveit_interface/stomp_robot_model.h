/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef STOMP_ROBOT_MODEL_H_
#define STOMP_ROBOT_MODEL_H_


#include <stomp_ros_interface/treefksolverjointposaxis_partial.hpp>
#include <stomp_ros_interface/stomp_collision_point.h>
#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <boost/shared_ptr.hpp>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/Constraints.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <planning_environment/models/robot_models.h>
#include <planning_models/kinematic_state.h>
#include <sensor_msgs/JointState.h>

#include <map>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <cstdlib>

namespace stomp_ros_interface
{

/**
 * \brief Contains all the information needed for STOMP planning.
 *
 * Initializes the robot models for STOMP
 * from the "robot_description" parameter, in the same way that
 * planning_environment::RobotModels does it.
 */
class StompRobotModel
{
public:

  /**
   * \brief Contains information about a single joint for STOMP planning
   */
  struct StompJoint
  {
    const KDL::Joint* kdl_joint_;                               /**< Pointer to the KDL joint in the tree */
    int kdl_joint_index_;                                       /**< Index for use in a KDL joint array */
    int stomp_joint_index_;                                     /**< Joint index for STOMP */
    std::string joint_name_;                                    /**< Name of the joint */
    std::string link_name_;                                     /**< Name of the corresponding link (from planning.yaml) */
    bool wrap_around_;                                          /**< Does this joint wrap-around? */
    bool has_joint_limits_;                                     /**< Are there joint limits? */
    double joint_limit_min_;                                    /**< Minimum joint angle value */
    double joint_limit_max_;                                    /**< Maximum joint angle value */
    double joint_update_limit_;                                 /**< Maximum amount the joint value can be updated in an iteration */
  };

  /**
   * \brief Contains information about a planning group
   */
  struct StompPlanningGroup
  {
    std::string name_;                                          /**< Name of the planning group */
    std::string end_effector_name_;                             /**< Name of the end effector */
    int end_effector_segment_index_;                            /**< segment id of the end effector */
    int num_joints_;                                            /**< Number of joints used in planning */
    std::vector<StompJoint> stomp_joints_;                      /**< Joints used in planning */
    //std::vector<std::string> link_names_;                       /**< Links used in planning */
    std::vector<std::string> collision_link_names_;             /**< Links used in collision checking */
    std::vector<StompCollisionPoint> collision_points_;         /**< Ordered list of collision checking points (from root to tip) */
    boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver_;           /**< Forward kinematics solver for the group */
    boost::shared_ptr<KDL::ChainIdSolver> id_solver_;           /**< Inverse dynamics solver for the group */
    KDL::Chain kdl_chain_;                                      /**< KDL Chain for the group */

    /**
     * Gets a random state vector within the joint limits
     */
    template <typename Derived>
    void getRandomState(Eigen::MatrixBase<Derived>& state_vec) const;

    /**
     * Adds the collision point to this planning group, if any of the joints in this group can
     * control the collision point in some way. Also converts the StompCollisionPoint::parent_joints
     * vector into group joint indexes
     */
    bool addCollisionPoint(StompCollisionPoint& collision_point, StompRobotModel& robot_model);

    /**
     * Gets the joint names in this group
     */
    std::vector<std::string> getJointNames() const;

    std::vector<double> getJointArrayFromJointState(const sensor_msgs::JointState& msg) const;
    std::vector<double> getJointArrayFromGoalConstraints(const arm_navigation_msgs::Constraints& msg) const;

    boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> getNewFKSolver() const;
  };

  StompRobotModel(ros::NodeHandle node_handle);
  virtual ~StompRobotModel();

  /**
   * \brief Initializes the robot models for STOMP
   *
   * \return true if successful, false if not
   */
  bool init(const std::string& reference_frame);

  /**
   * \brief Gets the planning group corresponding to the group name
   */
  const StompPlanningGroup* getPlanningGroup(const std::string& group_name) const;

  /**
   * \brief Gets the planning_environment::RobotModels class
   */
  //const planning_environment::RobotModels* getRobotModels() const;

  /**
   * \brief Gets the number of joints in the KDL tree
   */
  int getNumKDLJoints() const;

  /**
   * \brief Gets the KDL tree
   */
  const KDL::Tree* getKDLTree() const;

  /**
   * \brief Gets the KDL joint number from the URDF joint name
   *
   * \return -1 if the joint name is not found
   */
  int urdfNameToKdlNumber(const std::string& urdf_name) const;

  /**
   * \brief Gets the URDF joint name from the KDL joint number
   *
   * \return "" if the number does not have a name
   */
  const std::string kdlNumberToUrdfName(int kdl_number) const;

  //const KDL::TreeFkSolverJointPosAxis* getForwardKinematicsSolver() const;

  const std::string& getReferenceFrame() const;

  /**
   * \brief Takes in an std::vector of joint value messages, and writes them out into the KDL joint array.
   *
   * The template typename T needs to be an std::vector of some message which has an std::string "joint_name"
   * and a double array/vector "value".
   *
   * Names to KDL joint index mappings are performed using the given StompRobotModel.
   */
  template<typename T>
  void jointMsgToArray(T& msg_vector, Eigen::MatrixXd::RowXpr joint_array);

  /**
   * \brief Takes in an std::vector of joint value messages, and writes them out into the KDL joint array.
   *
   * The template typename T needs to be an std::vector of some message which has an std::string "joint_name"
   * and a double array/vector "value".
   *
   * Names to KDL joint index mappings are performed using the given StompRobotModel.
   */
  template<typename T>
  void jointMsgToArray(T& msg_vector, KDL::JntArray& joint_array);

  void jointStateToArray(const sensor_msgs::JointState &joint_state, KDL::JntArray& joint_array);

  void jointStateToArray(const sensor_msgs::JointState &joint_state, Eigen::MatrixXd::RowXpr joint_array);

  void getLinkCollisionPoints(std::string link_name, std::vector<StompCollisionPoint>& points);

  /**
   * \brief Gets the max value of radius+clearance for all the collision points
   */
  double getMaxRadiusClearance() const;

  /**
   * \brief Callback for information about objects attached to the robot
   */
  void attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr& attached_object);

  void generateAttachedObjectCollisionPoints(const arm_navigation_msgs::RobotState* robot_state);
  void generateLinkCollisionPoints();
  void populatePlanningGroupCollisionPoints();

  void publishCollisionPoints(ros::Publisher& vis_marker);

  std::vector<std::string> getJointNames() const;

private:
  ros::NodeHandle node_handle_,root_handle_;                                 /**< ROS Node handle */
  //planning_environment::CollisionSpaceMonitor* monitor_;
  //ros::Subscriber attached_object_subscriber_;                  /**< Attached object subscriber */

  boost::shared_ptr<planning_environment::RobotModels> robot_models_;
  std::vector<std::string> collision_links_;

  KDL::Tree kdl_tree_;                                          /**< The KDL tree of the entire robot */
  int num_kdl_joints_;                                          /**< Total number of joints in the KDL tree */
  std::map<std::string, std::string> joint_segment_mapping_;    /**< Joint -> Segment mapping for KDL tree */
  std::map<std::string, std::string> segment_joint_mapping_;    /**< Segment -> Joint mapping for KDL tree */
  std::vector<std::string> kdl_number_to_urdf_name_;            /**< Mapping from KDL joint number to URDF joint name */
  std::map<std::string, int> urdf_name_to_kdl_number_;          /**< Mapping from URDF joint name to KDL joint number */
  std::map<std::string, StompPlanningGroup> planning_groups_;   /**< Planning group information */
  KDL::TreeFkSolverJointPosAxisPartial *fk_solver_;                    /**< Forward kinematics solver for the tree */
  double collision_clearance_default_;                          /**< Default clearance for all collision links */
  std::string reference_frame_;                                 /**< Reference frame for all kinematics operations */
  std::map<std::string, std::vector<StompCollisionPoint> > link_collision_points_;    /**< Collision points associated with every link */
  std::map<std::string, std::vector<StompCollisionPoint> > link_attached_object_collision_points_;    /**< Collision points associated with the objects attached to every link */
  double max_radius_clearance_;                                 /**< Maximum value of radius + clearance for any of the collision points */
  std::map<std::string, arm_navigation_msgs::AttachedCollisionObject> attached_objects_;        /**< Map of links -> attached objects */

  void addCollisionPointsFromLink(std::string link_name, double clearance);
  //void addCollisionPointsFromAttachedObject(std::string link_name, mapping_msgs::AttachedCollisionObject& attached_object);
  void getLinkInformation(const std::string link_name, std::vector<int>& active_joints, int& segment_number);

//  void getActiveJointsSegmentNumberForLink(std::string link_name, 
};

/////////////////////////////// inline functions follow ///////////////////////////////////

inline const StompRobotModel::StompPlanningGroup* StompRobotModel::getPlanningGroup(const std::string& group_name) const
{
  std::map<std::string, StompRobotModel::StompPlanningGroup>::const_iterator it = planning_groups_.find(group_name);
  if (it == planning_groups_.end())
    return NULL;
  else
    return &(it->second);
}

//inline const planning_environment::RobotModels* StompRobotModel::getRobotModels() const
//{
//  return monitor_->getCollisionModels();
//}

inline int StompRobotModel::getNumKDLJoints() const
{
  return num_kdl_joints_;
}

inline const KDL::Tree* StompRobotModel::getKDLTree() const
{
  return &kdl_tree_;
}

inline int StompRobotModel::urdfNameToKdlNumber(const std::string& urdf_name) const
{
  std::map<std::string, int>::const_iterator it = urdf_name_to_kdl_number_.find(urdf_name);
  if (it!=urdf_name_to_kdl_number_.end())
    return it->second;
  else
    return -1;
}

inline const std::string StompRobotModel::kdlNumberToUrdfName(int kdl_number) const
{
  if (kdl_number<0 || kdl_number>=num_kdl_joints_)
    return std::string("");
  else
    return kdl_number_to_urdf_name_[kdl_number];
}

//inline const KDL::TreeFkSolverJointPosAxis* StompRobotModel::getForwardKinematicsSolver() const
//{
//  return fk_solver_;
//}

inline const std::string& StompRobotModel::getReferenceFrame() const
{
  return reference_frame_;
}

template <typename Derived>
void StompRobotModel::StompPlanningGroup::getRandomState(Eigen::MatrixBase<Derived>& state_vec) const
{
  for (int i=0; i<num_joints_; i++)
  {
    double min = stomp_joints_[i].joint_limit_min_;
    double max = stomp_joints_[i].joint_limit_max_;
    if (!stomp_joints_[i].has_joint_limits_)
    {
      min = -M_PI/2.0;
      max = M_PI/2.0;
    }
    state_vec(i) = ((((double)rand())/RAND_MAX) * (max-min)) + min;
  }
}

/**
 * \brief Takes in an std::vector of joint value messages, and writes them out into the KDL joint array.
 *
 * The template typename T needs to be an std::vector of some message which has an std::string "joint_name"
 * and a double array/vector "value".
 *
 * Names to KDL joint index mappings are performed using the given StompRobotModel.
 */
template<typename T>
void StompRobotModel::jointMsgToArray(T& msg_vector, Eigen::MatrixXd::RowXpr joint_array)
{
  for (typename T::iterator it=msg_vector.begin(); it!=msg_vector.end(); it++)
  {
    std::string name = it->joint_name;
    int kdl_number = urdfNameToKdlNumber(name);
    if (kdl_number>=0)
      joint_array(kdl_number) = it->value[0];   //@TODO we assume a single joint value per joint now
  }
}

template<typename T>
void StompRobotModel::jointMsgToArray(T& msg_vector, KDL::JntArray& joint_array)
{
  for (typename T::iterator it=msg_vector.begin(); it!=msg_vector.end(); it++)
  {
    std::string name = it->joint_name;
    int kdl_number = urdfNameToKdlNumber(name);
    if (kdl_number>=0)
      joint_array(kdl_number) = it->value[0];   //@TODO we assume a single joint value per joint now
  }
}

inline void StompRobotModel::jointStateToArray(const sensor_msgs::JointState &joint_state, KDL::JntArray& joint_array)
{
  for(unsigned int i=0; i < joint_state.name.size(); i++)
  {
    std::string name = joint_state.name[i];
    int kdl_number = urdfNameToKdlNumber(name);
    if (kdl_number>=0)
      joint_array(kdl_number) = joint_state.position[i]; 
  }
}

inline void StompRobotModel::jointStateToArray(const sensor_msgs::JointState &joint_state, Eigen::MatrixXd::RowXpr joint_array)
{
  for(unsigned int i=0; i < joint_state.name.size(); i++)
  {
    std::string name = joint_state.name[i];
    int kdl_number = urdfNameToKdlNumber(name);
    if (kdl_number>=0)
      joint_array(kdl_number) = joint_state.position[i]; 
  }
}


inline double StompRobotModel::getMaxRadiusClearance() const
{
  return max_radius_clearance_;
}

inline std::vector<std::string> StompRobotModel::getJointNames() const
{
  return kdl_number_to_urdf_name_;
}

} // namespace stomp
#endif /* STOMP_ROBOT_MODEL_H_ */
