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

void AvoidObstacles::init(const Constrained_IK * ik)
{
  Constraint::init(ik);
  num_robot_joints_ = ik_->getKin().numJoints();
  avoid_chain_ =   ik_->getKin().getSubChain(link_name_);
  num_inboard_joints_ = avoid_chain_.getNrOfJoints();
  jac_solver_ = new  KDL::ChainJntToJacSolver(avoid_chain_);
}

constrained_ik::ConstraintResults AvoidObstacles::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  AvoidObstacles::AvoidObstaclesData cdata(state);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

VectorXd AvoidObstacles::calcError(const AvoidObstacles::AvoidObstaclesData &cdata) const
{
  Eigen::Vector3d error_vector(0,0,0);
  CollisionRobotFCLDetailed::DistanceInfo dist_info;
  if(CollisionRobotFCLDetailed::getDistanceInfo(cdata.distance_map_, link_name_, dist_info))
  {
    double dist = dist_info.distance;
    double scale;
    if(dist > min_distance_)
    {
      scale = 1.0/(dist * dist);  // inverse square law
    }
    else
    {
      scale = 1/(min_distance_ * min_distance_ );
    }
    error_vector = scale*dist_info.avoidance_vector;
  }
  return  error_vector;
}

MatrixXd AvoidObstacles::calcJacobian(const AvoidObstacles::AvoidObstaclesData &cdata) const
{
  KDL::Jacobian link_jacobian(num_inboard_joints_); // 6xn Jacobian to link, then dist_info.link_point
  MatrixXd jacobian= MatrixXd::Zero(3, num_robot_joints_);  // 3xn jacobian to dist_info.link_point with just position, no rotation

  // calculate the link jacobian
  KDL::JntArray joint_array(num_inboard_joints_);
  for(int i=0; i<num_inboard_joints_; i++)   joint_array(i) = cdata.state_.joints(i);
  jac_solver_->JntToJac(joint_array, link_jacobian);// this computes a 6xn jacobian, we only need 3xn

  // use distance info to find reference point on link which is closest to a collision,
  // change the reference point of the link jacobian to that point
  CollisionRobotFCLDetailed::DistanceInfo dist_info;
  if(CollisionRobotFCLDetailed::getDistanceInfo(cdata.distance_map_, link_name_, dist_info))
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
  ROS_ASSERT(jacobian.cols()== num_robot_joints_);
  return jacobian;
}

bool AvoidObstacles::checkStatus(const AvoidObstacles::AvoidObstaclesData &cdata) const
{                               // returns true if its ok to stop with current ik conditions
  CollisionRobotFCLDetailed::DistanceInfo dist_info;
  CollisionRobotFCLDetailed::getDistanceInfo(cdata.distance_map_, link_name_, dist_info);
  if(dist_info.distance<min_distance_*5.0) return false;
  return true;
}

AvoidObstacles::AvoidObstaclesData::AvoidObstaclesData(const SolverState &state): ConstraintData(state)
{
  distance_map_ = state.collision_robot->distanceSelfDetailed(*state_.robot_state, state_.planning_scene->getAllowedCollisionMatrix());
}

} // end namespace constraints
} // end namespace constrained_ik
