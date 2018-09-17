/**
 * @file avoid_obstacles.cpp
 * @brief Constraint to avoid joint position limits.
 *
 * Using cubic velocity ramp, it pushes each joint away from its limits,
 * with a maximimum velocity of 2*threshold*(joint range).
 * Only affects joints that are within theshold of joint limit.
 *
 * @author clewis
 * @date June 2, 2015
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
#include "constrained_ik/constraints/avoid_obstacles.h"
#include <utility>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(constrained_ik::constraints::AvoidObstacles, constrained_ik::Constraint)

const double DEFAULT_WEIGHT = 1.0; /**< Default weight */
const double DEFAULT_MIN_DISTANCE = 0.1; /**< Default minimum obstacle distance allowed for convergence */
const double DEFAULT_AVOIDANCE_DISTANCE = 0.3; /**< Default distance at which to start avoiding the obstacle */
const double DEFAULT_AMPLITUDE = 0.3; /**< Default amplitude of the sigmoid error curve */
const double DEFAULT_SHIFT = 5.0; /**< Default shift for the sigmoid error curve */
const double DEFAULT_ZERO_POINT = 10; /**< Default zeros point for the sigmoid error curve */
const double DYNAMIC_WEIGHT_FUNCTION_CONSTANT = -13.86; /**< Default dynamic weight function constant */

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;
using namespace collision_detection;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::string;
using std::vector;

bool getDistanceInfo(const collision_detection::DistanceMap &distance_detailed, DistanceInfoMap &distance_info_map, const Eigen::Affine3d &tf)
{
  bool status = true;
  for (DistanceMap::const_iterator it = distance_detailed.begin(); it != distance_detailed.end(); ++it)
  {
    DistanceInfo dist_info;
    DistanceResultsData dist = static_cast<const DistanceResultsData>(it->second.front());
    if (dist.link_names[0] == it->first.first)
    {
      dist_info.nearest_obsticle = dist.link_names[1];
      dist_info.link_point = tf * dist.nearest_points[0];
      dist_info.obsticle_point = tf * dist.nearest_points[1];
      dist_info.avoidance_vector = dist_info.link_point - dist_info.obsticle_point;
      dist_info.avoidance_vector.normalize();
      dist_info.distance = dist.distance;
    }
    else if (dist.link_names[1] == it->first.first)
    {
      dist_info.nearest_obsticle = dist.link_names[0];
      dist_info.link_point = tf * dist.nearest_points[1];
      dist_info.obsticle_point = tf * dist.nearest_points[0];
      dist_info.avoidance_vector = dist_info.link_point - dist_info.obsticle_point;
      dist_info.avoidance_vector.normalize();
      dist_info.distance = dist.distance;
    }
    else
    {
      ROS_ERROR("getDistanceInfo was unable to find link after match!");
      status &= false;
    }

    distance_info_map.insert(std::make_pair(it->first.first, dist_info));
  }

  return status;
}

AvoidObstacles::LinkAvoidance::LinkAvoidance():
    weight_(DEFAULT_WEIGHT),
    min_distance_(DEFAULT_MIN_DISTANCE),
    avoidance_distance_(DEFAULT_AVOIDANCE_DISTANCE),
    amplitude_(DEFAULT_AMPLITUDE),
    jac_solver_(NULL),
    num_inboard_joints_(0),
    num_robot_joints_(0),
    num_obstacle_joints_(0)
{

}

AvoidObstacles::LinkAvoidance::LinkAvoidance(std::string link_name): LinkAvoidance() {link_name_ = link_name;}

void AvoidObstacles::init(const Constrained_IK * ik)
{
  Constraint::init(ik);

  if (link_names_.size() == 0)
  {
    ik_->getLinkNames(link_names_);
    ROS_WARN("Avoid Obstacles: No links were specified therefore using all links in kinematic chain.");
  }
  
  for (std::map<std::string, LinkAvoidance>::iterator it = links_.begin(); it != links_.end(); ++it)
  {
    it->second.num_robot_joints_ = ik_->getKin().numJoints();
    if (!ik_->getKin().getSubChain(it->second.link_name_, it->second.avoid_chain_))
    {
      ROS_ERROR_STREAM("Failed to initialize Avoid Obstalces constraint because"
                       "it failed to create a KDL chain between URDF links: '" <<
                       ik_->getKin().getRobotBaseLinkName() << "' and '" << it->second.link_name_ <<"'");
      initialized_ = false;
      return;
    }
    it->second.num_inboard_joints_ = it->second.avoid_chain_.getNrOfJoints();
    it->second.jac_solver_ = new  KDL::ChainJntToJacSolver(it->second.avoid_chain_);
  }

  std::vector<const robot_model::LinkModel*> tmp = ik_->getKin().getJointModelGroup()->getLinkModels();
  for (std::vector<const robot_model::LinkModel*>::const_iterator it = tmp.begin(); it < tmp.end(); ++it)
  {
    std::vector<std::string>::iterator name_it = std::find(link_names_.begin(), link_names_.end(), (*it)->getName());
    if (name_it != link_names_.end())
      link_models_.insert(*it);
  }
}

void AvoidObstacles::loadParameters(const XmlRpc::XmlRpcValue &constraint_xml)
{
  XmlRpc::XmlRpcValue local_xml = constraint_xml;
  std::vector<std::string> link_names;
  if (getParam(local_xml, "link_names", link_names))
  {    
    std::vector<double> amplitude, minimum_distance, avoidance_distance, weight;
    if (getParam(local_xml, "amplitude", amplitude))
    {
      if (link_names.size()!=amplitude.size())
      {
        ROS_WARN("Abstacle Avoidance: amplitude memebr must be same size array as link_names member, default parameters will be used.");
        amplitude.clear();
      }
    }
    else
    {
      ROS_WARN("Abstacle Avoidance: Unable to retrieve amplitude member, default parameter will be used.");
    }

    if (getParam(local_xml, "minimum_distance", minimum_distance))
    {
      if (link_names.size()!=minimum_distance.size())
      {
        ROS_WARN("Abstacle Avoidance: minimum_distance memebr must be same size array as link_names member, default parameters will be used.");
        minimum_distance.clear();
      }
    }
    else
    {
      ROS_WARN("Abstacle Avoidance: Unable to retrieve minimum_distance member, default parameter will be used.");
    }

    if (getParam(local_xml, "avoidance_distance", avoidance_distance))
    {
      if (link_names.size()!=avoidance_distance.size())
      {
        ROS_WARN("Abstacle Avoidance: avoidance_distance memebr must be same size array as link_names member, default parameters will be used.");
        avoidance_distance.clear();
      }
    }
    else
    {
      ROS_WARN("Abstacle Avoidance: Unable to retrieve avoidance_distance member, default parameter will be used.");
    }

    if (getParam(local_xml, "weights", weight))
    {
      if (link_names.size()!=weight.size())
      {
        ROS_WARN("Abstacle Avoidance: weights member must be same size array as link_names member, default parameters will be used.");
        weight.clear();
      }
    }
    else
    {
      ROS_WARN("Abstacle Avoidance: Unable to retrieve weight member, default parameter will be used.");
    }


    for (int i=0; i<link_names.size(); ++i)
    {
      addAvoidanceLink(link_names[i]);
      if (!amplitude.empty())
      {
        setAmplitude(link_names[i], amplitude[i]);
      }
      if (!minimum_distance.empty())
      {
        setMinDistance(link_names[i], minimum_distance[i]);
      }
      if (!avoidance_distance.empty())
      {
        setAvoidanceDistance(link_names[i], avoidance_distance[i]);
      }
      if (!weight.empty())
      {
        setWeight(link_names[i], weight[i]);
      }
    }
  }
  else
  {
    ROS_WARN("Abstacle Avoidance: Unable to retrieve link_names member, default parameter will be used.");
  }
}

ConstraintResults AvoidObstacles::evalConstraint(const SolverState &state) const
{
  ConstraintResults output;
  AvoidObstaclesData cdata(state, this);
  double dynamic_weight;
  for (std::map<std::string, LinkAvoidance>::const_iterator it = links_.begin(); it != links_.end(); ++it)
  {
    DistanceInfoMap::const_iterator dit = cdata.distance_info_map_.find(it->second.link_name_);
    if (dit != cdata.distance_info_map_.end() && dit->second.distance > 0)
    {
      dynamic_weight = std::exp(DYNAMIC_WEIGHT_FUNCTION_CONSTANT * (std::abs(
          dit->second.distance-cdata.distance_res_.minimum_distance.distance)/distance_threshold_));
      constrained_ik::ConstraintResults tmp;
      tmp.error = calcError(cdata, it->second) * it->second.weight_ * dynamic_weight;
      tmp.jacobian = calcJacobian(cdata, it->second)  * it->second.weight_ * dynamic_weight;
      tmp.status = checkStatus(cdata, it->second);
      output.append(tmp);
    }
  }

  return output;
}

VectorXd AvoidObstacles::calcError(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{
  Eigen::VectorXd dist_err;
  DistanceInfoMap::const_iterator it;

  dist_err.resize(1,1);
  it = cdata.distance_info_map_.find(link.link_name_);
  if (it != cdata.distance_info_map_.end() && it->second.distance > 0)
  {
    double dist = it->second.distance;
    double scale_x = link.avoidance_distance_/(DEFAULT_ZERO_POINT + DEFAULT_SHIFT);
    double scale_y = link.amplitude_;
    dist_err(0, 0) = scale_y/(1.0 + std::exp((dist/scale_x) - DEFAULT_SHIFT));
  }
  else
  {
    ROS_DEBUG("Unable to retrieve distance info, couldn't find link with that name %s", link.link_name_.c_str());
  }
  return  dist_err;
}

MatrixXd AvoidObstacles::calcJacobian(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{
  KDL::Jacobian link_jacobian(link.num_inboard_joints_); // 6xn Jacobian to link, then dist_info.link_point
  MatrixXd jacobian;

  // use distance info to find reference point on link which is closest to a collision,
  // change the reference point of the link jacobian to that point
  DistanceInfoMap::const_iterator it;
  jacobian.setZero(1, link.num_robot_joints_);
  it = cdata.distance_info_map_.find(link.link_name_);
  if (it != cdata.distance_info_map_.end() && it->second.distance > 0)
  {
    KDL::JntArray joint_array(link.num_inboard_joints_);
    for(int i=0; i<link.num_inboard_joints_; i++)   joint_array(i) = cdata.state_.joints(i);
    link.jac_solver_->JntToJac(joint_array, link_jacobian);// this computes a 6xn jacobian, we only need 3xn

    // change the referece point to the point on the link closest to a collision
    KDL::Vector link_point(it->second.link_point.x(), it->second.link_point.y(), it->second.link_point.z());
    link_jacobian.changeRefPoint(link_point);
    
    MatrixXd j_tmp;
    basic_kin::BasicKin::KDLToEigen(link_jacobian,j_tmp);
    
    // The jacobian to improve distance only requires 1 redundant degree of freedom
    // so we project the jacobian onto the avoidance vector.
    jacobian.block(0, 0, 1, j_tmp.cols()) = it->second.avoidance_vector.transpose() * j_tmp.topRows(3);
  }
  else
  {
    ROS_DEBUG("Unable to retrieve distance info, couldn't find link with that name %s", link.link_name_.c_str());
  }

  return jacobian;
}

bool AvoidObstacles::checkStatus(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{                               // returns true if its ok to stop with current ik conditions
  DistanceInfoMap::const_iterator it;

  it = cdata.distance_info_map_.find(link.link_name_);
  if (it != cdata.distance_info_map_.end())
  {
    if(it->second.distance<link.min_distance_) return false;
  }
  else
  {
    ROS_DEBUG("couldn't find link with that name %s", link.link_name_.c_str());
  }

  return true;
}

AvoidObstacles::AvoidObstaclesData::AvoidObstaclesData(const SolverState &state, const AvoidObstacles *parent): ConstraintData(state), parent_(parent)
{
  DistanceRequest distance_req;
  distance_req.enable_nearest_points = true;
  distance_req.enable_signed_distance = true;
  distance_req.active_components_only = &parent_->link_models_;
  distance_req.acm = &state_.planning_scene->getAllowedCollisionMatrix();
  distance_req.distance_threshold = parent_->distance_threshold_;
  distance_req.group_name = state.group_name;
  distance_res_.clear();
  
  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;

  state.collision_robot->distanceSelf(distance_req, distance_res_, *state_.robot_state);
  state.collision_world->distanceRobot(distance_req, distance_res_, *state_.collision_robot, *state_.robot_state);
  if (distance_res_.collision)
  {
    collision_req.distance = false;
    collision_req.contacts = true;
    collision_req.max_contacts = parent_->link_models_.size() * 2.0;
    state.collision_robot->checkSelfCollision(collision_req, collision_res, *state_.robot_state, state_.planning_scene->getAllowedCollisionMatrix());
    state.collision_world->checkRobotCollision(collision_req, collision_res, *state.collision_robot, *state_.robot_state, state_.planning_scene->getAllowedCollisionMatrix());
  }
  Eigen::Affine3d tf = state_.robot_state->getGlobalLinkTransform(parent_->ik_->getKin().getRobotBaseLinkName()).inverse();
  getDistanceInfo(distance_res_.distances, distance_info_map_, tf);
}

} // end namespace constraints
} // end namespace constrained_ik
