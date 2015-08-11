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
#include "constrained_ik/constraints/avoid_obstacles.h"
#include <utility>
#include <ros/ros.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::string;
using std::vector;

AvoidObstacles::AvoidObstacles(std::vector<std::string> &link_names):  Constraint(), link_names_(link_names)
{
  for (std::vector<std::string>::const_iterator it = link_names.begin(); it < link_names.end(); ++it)
    links_.insert(std::make_pair(*it, LinkAvoidance(*it)));
}

void AvoidObstacles::init(const Constrained_IK * ik)
{
  Constraint::init(ik);
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

ConstraintResults AvoidObstacles::evalConstraint(const SolverState &state) const
{
  ConstraintResults output;
  AvoidObstaclesData cdata(state, this);
  for (std::map<std::string, LinkAvoidance>::const_iterator it = links_.begin(); it != links_.end(); ++it)
  {
    constrained_ik::ConstraintResults tmp;
    tmp.error = calcError(cdata, it->second);
    tmp.jacobian = calcJacobian(cdata, it->second);
    tmp.status = checkStatus(cdata, it->second);
    output.append(tmp);
  }

  return output;
}

VectorXd AvoidObstacles::calcError(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{
  Eigen::VectorXd error_vector;
  CollisionRobotFCLDetailed::DistanceInfoMap::const_iterator it;

  it = cdata.distance_info_map_.find(link.link_name_);
  if (it != cdata.distance_info_map_.end())
  {
    double dist = it->second.distance;
    double shift = 5.0;
    double zero_point = 20.0 - shift;
    double scale_x = link.min_distance_/zero_point;
    double scale_y = link.amplitude_;
    double scale = scale_y/(1.0 + std::exp((dist/scale_x) - shift));
    error_vector = scale*it->second.avoidance_vector;
  }
  else
  {
    ROS_DEBUG("Unable to retrieve distance info, couldn't find link with that name %s", link.link_name_.c_str());
  }
  return  error_vector;
}

MatrixXd AvoidObstacles::calcJacobian(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{
  KDL::Jacobian link_jacobian(link.num_inboard_joints_); // 6xn Jacobian to link, then dist_info.link_point
  MatrixXd jacobian;

  // use distance info to find reference point on link which is closest to a collision,
  // change the reference point of the link jacobian to that point
  CollisionRobotFCLDetailed::DistanceInfoMap::const_iterator it;
  it = cdata.distance_info_map_.find(link.link_name_);
  if (it != cdata.distance_info_map_.end())
  {
    jacobian.setZero(3, link.num_robot_joints_); // 3xn jacobian to dist_info.link_point with just position, no rotation
    // calculate the link jacobian
    KDL::JntArray joint_array(link.num_inboard_joints_);
    for(int i=0; i<link.num_inboard_joints_; i++)   joint_array(i) = cdata.state_.joints(i);
    link.jac_solver_->JntToJac(joint_array, link_jacobian);// this computes a 6xn jacobian, we only need 3xn

    // change the referece point to the point on the link closest to a collision
    KDL::Vector link_point(it->second.link_point.x(), it->second.link_point.y(), it->second.link_point.z());
    link_jacobian.changeRefPoint(link_point);

    // copy the upper 3xn portion of full sized link jacobian into positional jacobain (3xm)
    for(int i=0; i<3; i++)
    {
      for(int j=0; j<(int) link_jacobian.columns(); j++)
      {
        jacobian(i,j) = link_jacobian(i,j);
      }
    }
    ROS_ASSERT(jacobian.rows()==3);
    ROS_ASSERT(jacobian.cols()== link.num_robot_joints_);
  }
  else
  {
    ROS_DEBUG("Unable to retrieve distance info, couldn't find link with that name %s", link.link_name_.c_str());
  }

  return jacobian;
}

bool AvoidObstacles::checkStatus(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{                               // returns true if its ok to stop with current ik conditions
  CollisionRobotFCLDetailed::DistanceInfoMap::const_iterator it;

  it = cdata.distance_info_map_.find(link.link_name_);
  if (it != cdata.distance_info_map_.end())
  {
    if(it->second.distance<link.min_distance_*5.0) return false;
  }
  else
  {
    ROS_DEBUG("couldn't find link with that name %s", link.link_name_.c_str());
  }

  return true;
}

AvoidObstacles::AvoidObstaclesData::AvoidObstaclesData(const SolverState &state, const AvoidObstacles *parent): ConstraintData(state), parent_(parent)
{
  distance_map_ = state.collision_robot->distanceSelfDetailed(*state_.robot_state, state_.planning_scene->getAllowedCollisionMatrix(), parent_->link_models_);
  Eigen::Affine3d tf = parent_->ik_->getKin().getRobotBaseInWorld().inverse();
  CollisionRobotFCLDetailed::getDistanceInfo(distance_map_, distance_info_map_, tf);
}

} // end namespace constraints
} // end namespace constrained_ik
