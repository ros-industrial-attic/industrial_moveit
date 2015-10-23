/**
 * @file avoid_obstacles.h
 * @brief Constraint to avoid obstacles
 *
 *
 * @author clewis
 * @date June 1 2015
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
#ifndef AVOID_OBSTACLES_H_
#define AVOID_OBSTACLES_H_

#include "constrained_ik/constraint.h"
#include "constrained_ik/constrained_ik.h"
#include <constrained_ik/collision_robot_fcl_detailed.h>
#include <vector>
#include <algorithm>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace constrained_ik
{
namespace constraints
{

/**
 * @brief Constraint to avoid collisions
 *
 */
class AvoidObstacles: public Constraint
{
protected:
  struct LinkAvoidance
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LinkAvoidance(std::string link_name);
    LinkAvoidance() {}
    virtual ~LinkAvoidance()
    {
      delete jac_solver_;
    }

    double weight_; /**< importance weight applied to this avoidance constraint */
    double min_distance_; /**< minimum obstacle distance allowed for convergence */
    double avoidance_distance_; /**< distance at which to start avoiding the obstacle */
    double amplitude_; /**< The amplitude of the sigmoid error curve */
    int num_robot_joints_; /**< number of joints in the whole robot*/
    int num_obstacle_joints_; /**< number of joints inboard to the obstacle link */
    std::string link_name_; /**< the name of the link that is to avoid obstacles */
    KDL::Chain avoid_chain_; /**< the kinematic chain from base to the obstacle avoidance link */
    int num_inboard_joints_; /**< number of joints in the inboard chain */
    KDL::Vector link_point_; /**< vector to point on link closest to an obstacle */
    KDL::ChainJntToJacSolver * jac_solver_; /**< a KDL object for computing jacobians */
    KDL::Vector obstacle_point_; /**< vector to point on link closest to an obstacle */
  };

  std::map<std::string, LinkAvoidance> links_;
  std::vector<std::string> link_names_;
  std::set<const robot_model::LinkModel *> link_models_;

  LinkAvoidance* getLinkData(std::string link_name)
  {
    std::map<std::string, LinkAvoidance>::iterator it;
    it = links_.find(link_name);
    if(it != links_.end())
    {
      return &(it->second);
    }
    else
    {
      ROS_WARN_STREAM("Failed to retrieve avoidance data for link: " << link_name);
      return NULL;
    }
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct AvoidObstaclesData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const constraints::AvoidObstacles* parent_;
    CollisionRobotFCLDetailed::DistanceDetailedMap distance_map_;
    CollisionRobotFCLDetailed::DistanceInfoMap distance_info_map_;

    AvoidObstaclesData(const constrained_ik::SolverState &state, const constraints::AvoidObstacles* parent);
  };

  /**
   * @brief constructor
   * @param link_name name of link which should avoid obstacles
   */
  AvoidObstacles() {}
  virtual ~AvoidObstacles() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Creates Jacobian for avoiding a collision with link closest to a collision
   * @param cdata, The constraint specific data.
   * @return Jacobian scaled by weight
   */
  virtual Eigen::MatrixXd calcJacobian(const AvoidObstaclesData &cdata, const LinkAvoidance &link) const;

  /**
   * @brief Creates vector representing velocity error term
   * corresponding to calcJacobian()
   * @param cdata, The constraint specific data.
   * @return VectorXd of joint velocities for obstacle avoidance
   */
  virtual Eigen::VectorXd calcError(const AvoidObstaclesData &cdata, const LinkAvoidance &link) const;

  /**
   * @brief Checks termination criteria
   * There are no termination criteria for this constraint
   * @param cdata, The constraint specific data.
   * @return True
   */
  virtual bool checkStatus(const AvoidObstaclesData &cdata, const LinkAvoidance &link) const;

  /**
   * @brief Initialize constraint (overrides Constraint::init)
   * Should be called before using class.
   * @param ik Pointer to Constrained_IK used for base-class init
   */
  virtual void init(const Constrained_IK * ik);

  /**
   * @brief Load constraint parameters from XmlRpc::XmlRpcValue
   * @param constraint_xml XmlRpc::XmlRpcValue
   */
  virtual void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml);

  /**
   * @brief getter for link weight_
   * @param link_name Name of link to get weight_
   * @return weight_, On error -1.0 is returned
   */
  double getWeight(const std::string &link_name)
  {
    LinkAvoidance *link;
    if(getLinkData(link_name))
      return link->weight_;
    else
      return -1.0;
  }

  /**
   * @brief setter for link weight_
   * @param link_name Name of link to set weight_
   * @param weight Value to set weight_ to
   */
  void setWeight(const std::string &link_name, const double &weight)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      link->weight_ = weight;
  }

  /**
   * @brief getter for link min_distance_
   * @param link_name Name of link to get min_distance_
   * @return min_distance_, On error -1.0 is returned
   */
  double getMinDistance(const std::string &link_name)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      return link->min_distance_;
    else
      return -1.0;
  }

  /**
   * @brief setter for link min_distance_
   * @param link_name Name of link to set min_distance_
   * @param weight Value to set min_distance_ to
   */
  void setMinDistance(const std::string &link_name, const double &min_distance)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      link->min_distance_ = min_distance;
  }

  /**
   * @brief getter for link amplitude
   * @param link_name Name of link to get amplitude data
   * @return amplitude_, On error -1.0 is returned
   */
  double getAmplitude(const std::string &link_name)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      return link->amplitude_;
    else
      return -1.0;
  }

  /**
   * @brief setter for link amplitude
   * @param link_name Name of link to set amplitude_
   * @param amplitude Value to set amplitude_ to
   */
  void setAmplitude(const std::string &link_name, const double &amplitude)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      link->amplitude_ = amplitude;
  }
  
  /**
   * @brief getter for link avoidance distance
   * @param link_name Name of link to get avoidance distance data
   * @return avoidance_distance_, On error -1.0 is returned
   */
  double getAvoidanceDistance(const std::string &link_name)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      return link->avoidance_distance_;
    else
      return -1.0;
  }

  /**
   * @brief setter for link avoidance distance
   * @param link_name Name of link to set avoidance_distance_
   * @param avoidance_distance Value to set avoidance_distance_ to
   */
  void setAvoidanceDistance(const std::string &link_name, const double &avoidance_distance)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      link->avoidance_distance_ = avoidance_distance;
  }
  
  /**
   * @brief getter for obstacle avoidance links
   * @return link_names_
   */
  std::vector<std::string> getAvoidanceLinkNames()
  {
    return link_names_;
  }

  /**
   * @brief setter for obstacle avoidance links
   * @param link_name Name of link to set link_names_
   */
  void setAvoidanceLinks(const std::vector<std::string> &link_names)
  {
    link_names_ = link_names;
    links_.clear();
    
    for (std::vector<std::string>::const_iterator it = link_names.begin(); it < link_names.end(); ++it)
      links_.insert(std::make_pair(*it, LinkAvoidance(*it)));
  }

  /**
   * @brief Adds an obstacle avoidance link
   * @param link_name Name of link to add to link_names_
   */
  void addAvoidanceLink(const std::string &link_name)
  {
    if (std::find(link_names_.begin(), link_names_.end(), link_name) == link_names_.end())
    {
      links_.insert(std::make_pair(link_name, LinkAvoidance(link_name)));
      link_names_.push_back(link_name);
    }
    else
    {
      ROS_WARN("Tried to add an avoidance link that already exist.");
    }
  }

};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* AVOID_OBSTACLES_H_ */
