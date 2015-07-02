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

AvoidObstacles::AvoidObstacles(std::vector<std::string> link_names):  Constraint()
{
//  for (int i = 0; i < link_names.size(); i++)
//  {
//    std::string link_name = link_names[i];
//    LinkAvoidance link(link_name);
//    ROS_INFO_STREAM("Make pair for: " << link_name);
//  }
  for (std::vector<std::string>::const_iterator it = link_names.begin(); it < link_names.end(); ++it)
  {
    //std::string link_name = *it;
    //LinkAvoidance link(link_name);
    links_.insert(std::make_pair(*it, LinkAvoidance(*it)));
  }
}

void AvoidObstacles::init(const Constrained_IK * ik)
{
  Constraint::init(ik);
  for (std::map<std::string, LinkAvoidance>::iterator it = links_.begin(); it != links_.end(); ++it)
  {
    it->second.num_robot_joints_ = ik_->getKin().numJoints();
    it->second.avoid_chain_ =   ik_->getKin().getSubChain(it->second.link_name_);
    it->second.num_inboard_joints_ = it->second.avoid_chain_.getNrOfJoints();
    it->second.jac_solver_ = new  KDL::ChainJntToJacSolver(it->second.avoid_chain_);
  }
}

ConstraintResults AvoidObstacles::evalConstraint(const SolverState &state) const
{
  ConstraintResults output;
  AvoidObstaclesData cdata(state);
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
  Eigen::Vector3d error_vector(0,0,0);
  CollisionRobotFCLDetailed::DistanceInfo dist_info;
  if(CollisionRobotFCLDetailed::getDistanceInfo(cdata.distance_map_, link.link_name_, dist_info))
  {
    double dist = dist_info.distance;
    double scale;
    if(dist > link.min_distance_)
    {
      scale = 1.0/(dist * dist);  // inverse square law
    }
    else
    {
      scale = 1/(link.min_distance_ * link.min_distance_ );
    }
    error_vector = scale*dist_info.avoidance_vector;
  }
  return  error_vector;
}

MatrixXd AvoidObstacles::calcJacobian(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{
  KDL::Jacobian link_jacobian(link.num_inboard_joints_); // 6xn Jacobian to link, then dist_info.link_point
  MatrixXd jacobian= MatrixXd::Zero(3, link.num_robot_joints_);  // 3xn jacobian to dist_info.link_point with just position, no rotation

  // calculate the link jacobian
  KDL::JntArray joint_array(link.num_inboard_joints_);
  for(int i=0; i<link.num_inboard_joints_; i++)   joint_array(i) = cdata.state_.joints(i);
  link.jac_solver_->JntToJac(joint_array, link_jacobian);// this computes a 6xn jacobian, we only need 3xn

  // use distance info to find reference point on link which is closest to a collision,
  // change the reference point of the link jacobian to that point
  CollisionRobotFCLDetailed::DistanceInfo dist_info;
  if(CollisionRobotFCLDetailed::getDistanceInfo(cdata.distance_map_, link.link_name_, dist_info))
  {
    // change the referece point to the point on the link closest to a collision
    KDL::Vector link_point(dist_info.link_point.x(), dist_info.link_point.y(), dist_info.link_point.z());
    link_jacobian.changeRefPoint(link_point);

    // copy the upper 3xn portion of full sized link jacobian into positional jacobain (3xm)
    for(int i=0; i<3; i++)
    {
      for(int j=0; j<(int) link_jacobian.columns(); j++)
      {
        jacobian(i,j) = link_jacobian(i,j);
      }
    }
  }
  else
  {
    ROS_ERROR("couldn't get distance info");
  }
  ROS_ASSERT(jacobian.rows()==3);
  ROS_ASSERT(jacobian.cols()== link.num_robot_joints_);
  return jacobian;
}

bool AvoidObstacles::checkStatus(const AvoidObstacles::AvoidObstaclesData &cdata, const LinkAvoidance &link) const
{                               // returns true if its ok to stop with current ik conditions
  CollisionRobotFCLDetailed::DistanceInfo dist_info;
  CollisionRobotFCLDetailed::getDistanceInfo(cdata.distance_map_, link.link_name_, dist_info);
  if(dist_info.distance<link.min_distance_*5.0) return false;
  return true;
}

AvoidObstacles::AvoidObstaclesData::AvoidObstaclesData(const SolverState &state): ConstraintData(state)
{
  distance_map_ = state.collision_robot->distanceSelfDetailed(*state_.robot_state, state_.planning_scene->getAllowedCollisionMatrix());
}

} // end namespace constraints
} // end namespace constrained_ik
