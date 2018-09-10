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
#ifndef AVOID_OBSTACLES_H_
#define AVOID_OBSTACLES_H_

#include "constrained_ik/constraint.h"
#include "constrained_ik/constrained_ik.h"
#include <moveit/collision_detection/collision_common.h>
#include <vector>
#include <algorithm>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace constrained_ik
{
namespace constraints
{

/** @brief Contains distance information in the planning frame queried from getDistanceInfo() */
struct DistanceInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string nearest_obsticle;     /**< The link name for nearest obsticle/link to request link. */
  Eigen::Vector3d link_point;       /**< Point on request link */
  Eigen::Vector3d obsticle_point;   /**< Point on nearest link to requested link */
  Eigen::Vector3d avoidance_vector; /**< Normilized Vector created by nearest points */
  double distance;                  /**< Distance between nearest points */
};
typedef std::map<std::string, DistanceInfo> DistanceInfoMap;

/**
 * @brief getDistanceInfo
 * @param distance_detailed Detailed Distance Map
 * @param distance_info_map Stores the distance information for each link in DistanceDetailedMap
 * @param tf This allows for a transformation to be applied the distance data since it is always returned in the world frame from fcl.
 * @return bool, true if succesfully converted DistanceDetailedMap to DistanceInfoMap
 */
bool getDistanceInfo(const collision_detection::DistanceMap &distance_detailed, DistanceInfoMap &distance_info_map, const Eigen::Affine3d &tf);

/**
 * @class constrained_ik::constraints::AvoidObstacles
 * @brief Constraint to avoid obstacles
 *
 * @par Examples:
 * All examples are located here @ref avoid_joint_limits_example
 */
class AvoidObstacles: public Constraint
{
protected:
  /** @brief Link avoidance data for a single link */
  struct LinkAvoidance
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief Constructor
     * @param link_name name of link
     */
    LinkAvoidance(std::string link_name);

    LinkAvoidance();

    virtual ~LinkAvoidance() {delete jac_solver_;}

    double weight_; /**< importance weight applied to this avoidance constraint */
    double min_distance_; /**< minimum obstacle distance allowed for convergence */
    double avoidance_distance_; /**< distance at which to start avoiding the obstacle */
    double amplitude_; /**< amplitude of the sigmoid error curve */
    int num_robot_joints_; /**< number of joints in the whole robot*/
    int num_obstacle_joints_; /**< number of joints inboard to the obstacle link */
    std::string link_name_; /**< the name of the link that is to avoid obstacles */
    KDL::Chain avoid_chain_; /**< the kinematic chain from base to the obstacle avoidance link */
    int num_inboard_joints_; /**< number of joints in the inboard chain */
    KDL::Vector link_point_; /**< vector to point on link closest to an obstacle */
    KDL::ChainJntToJacSolver * jac_solver_; /**< a KDL object for computing jacobians */
    KDL::Vector obstacle_point_; /**< vector to point on link closest to an obstacle */
  };

  std::map<std::string, LinkAvoidance> links_; /**< @brief map from link name to its avoidance data */
  std::vector<std::string> link_names_; /**< @brief list of links that should avoid obstacles */
  std::set<const robot_model::LinkModel *> link_models_; /**< @brief a set of LinkModel for each link in link_names_ */
  double distance_threshold_; /**< @brief a distance threshold used to speed up distance queries */

  /**
   * @brief Get a links avoidance data
   * @param link_name name of the link
   * @return LinkAvoidance object for named link
   */
  LinkAvoidance* getLinkData(const std::string link_name)
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

  /**
   * @brief Get a links avoidance data
   * @param link_name name of the link
   * @return LinkAvoidance object for named link
   */
  const LinkAvoidance * const getLinkData(const std::string link_name) const
  {
    std::map<std::string, LinkAvoidance>::const_iterator it;
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
  /** @brief This structure stores constraint data */
  struct AvoidObstaclesData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const constraints::AvoidObstacles* parent_; /**< pointer to parent class AvoidObstacles */
    collision_detection::DistanceResult distance_res_; /**< stores the minimum distance results */
    collision_detection::DistanceMap distance_map_; /**< map of link names to its distance results */
    DistanceInfoMap distance_info_map_; /**< map of link names to distance information */

    /** @brief See base class for documentation */
    AvoidObstaclesData(const constrained_ik::SolverState &state, const constraints::AvoidObstacles* parent);
  };

  AvoidObstacles() {}

  /**
   * @brief Initialize constraint (overrides Constraint::init)
   * Should be called before using class.
   * @param ik Pointer to Constrained_IK used for base-class init
   */
  void init(const Constrained_IK * ik) override;

  /** @brief see base class for documentation*/
  constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const override;

  /** @brief see base class for documentation*/
  void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml) override;

  /**
   * @brief Creates Jacobian for avoiding a collision with link closest to a collision
   * @param cdata The constraint specific data.
   * @param link The link to process
   * @return Jacobian scaled by weight
   */
  virtual Eigen::MatrixXd calcJacobian(const AvoidObstaclesData &cdata, const LinkAvoidance &link) const;

  /**
   * @brief Creates vector representing velocity error term
   * corresponding to calcJacobian()
   * @param cdata The constraint specific data.
   * @param link The link to process
   * @return VectorXd of joint velocities for obstacle avoidance scaled by weight
   */
  virtual Eigen::VectorXd calcError(const AvoidObstaclesData &cdata, const LinkAvoidance &link) const;

  /**
   * @brief Checks termination criteria
   * There are no termination criteria for this constraint
   * @param cdata The constraint specific data.
   * @param link The link to process
   * @return True
   */
  virtual bool checkStatus(const AvoidObstaclesData &cdata, const LinkAvoidance &link) const;

  /**
   * @brief getter for link weight_
   * @param link_name Name of link to get weight_
   * @return weight_ On error -1.0 is returned
   */
  virtual double getWeight(const std::string &link_name) const
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
  virtual void setWeight(const std::string &link_name, const double &weight)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      link->weight_ = weight;
  }

  /**
   * @brief getter for link min_distance_
   * @param link_name Name of link to get min_distance_
   * @return min_distance_ On error -1.0 is returned
   */
  virtual double getMinDistance(const std::string &link_name) const
  {
    const LinkAvoidance * const link = getLinkData(link_name);
    if(link)
      return link->min_distance_;
    else
      return -1.0;
  }

  /**
   * @brief setter for link min_distance_
   * @param link_name Name of link to set min_distance_
   * @param min_distance Value to set min_distance_ to
   */
  virtual void setMinDistance(const std::string &link_name, const double &min_distance)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      link->min_distance_ = min_distance;
  }

  /**
   * @brief getter for link amplitude
   * @param link_name Name of link to get amplitude data
   * @return amplitude_ On error -1.0 is returned
   */
  virtual double getAmplitude(const std::string &link_name) const
  {
    const LinkAvoidance * const link = getLinkData(link_name);
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
  virtual void setAmplitude(const std::string &link_name, const double &amplitude)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
      link->amplitude_ = amplitude;
  }
  
  /**
   * @brief getter for link avoidance distance
   * @param link_name Name of link to get avoidance distance data
   * @return avoidance_distance_ On error -1.0 is returned
   */
  virtual double getAvoidanceDistance(const std::string &link_name) const
  {
    const LinkAvoidance * const link = getLinkData(link_name);
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
  virtual void setAvoidanceDistance(const std::string &link_name, const double &avoidance_distance)
  {
    LinkAvoidance* link = getLinkData(link_name);
    if(link)
    {
      link->avoidance_distance_ = avoidance_distance;
      updateDistanceThreshold();
    }
  }
  
  /**
   * @brief getter for obstacle avoidance links
   * @return link_names_
   */
  virtual std::vector<std::string> getAvoidanceLinkNames() const
  {
    return link_names_;
  }

  /**
   * @brief setter for obstacle avoidance links
   * @param link_names List of link names to set link_names_
   */
  virtual void setAvoidanceLinks(const std::vector<std::string> &link_names)
  {
    link_names_ = link_names;
    links_.clear();
    
    for (std::vector<std::string>::const_iterator it = link_names.begin(); it < link_names.end(); ++it)
    {
      links_.insert(std::make_pair(*it, LinkAvoidance(*it)));
      updateDistanceThreshold();
    }
  }

  /**
   * @brief Adds an obstacle avoidance link
   * @param link_name Name of link to add to link_names_
   */
  virtual void addAvoidanceLink(const std::string &link_name)
  {
    if (std::find(link_names_.begin(), link_names_.end(), link_name) == link_names_.end())
    {
      links_.insert(std::make_pair(link_name, LinkAvoidance(link_name)));
      link_names_.push_back(link_name);
      updateDistanceThreshold();
    }
    else
    {
      ROS_WARN("Tried to add an avoidance link that already exist.");
    }
  }

  /**
   * @brief This updates the maximum distance threshold
   */
  virtual void updateDistanceThreshold()
  {
    distance_threshold_ = 0;
    for (std::map<std::string, LinkAvoidance>::const_iterator it = links_.begin(); it != links_.end(); it++)
    {
      if (it->second.avoidance_distance_ > distance_threshold_)
      {
        distance_threshold_ = it->second.avoidance_distance_;
      }
    }
  }

};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* AVOID_OBSTACLES_H_ */
