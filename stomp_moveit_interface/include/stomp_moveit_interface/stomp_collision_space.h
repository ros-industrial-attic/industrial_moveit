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

#ifndef STOMP_COLLISION_SPACE_H_
#define STOMP_COLLISION_SPACE_H_

#include <arm_navigation_msgs/CollisionMap.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/RobotState.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <stomp_ros_interface/stomp_collision_point.h>
#include <stomp_ros_interface/stomp_robot_model.h>
#include <Eigen/Core>
#include <distance_field/distance_field.h>
#include <distance_field/propagation_distance_field.h>
#include <distance_field/pf_distance_field.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/monitors/collision_space_monitor.h>

namespace stomp_ros_interface
{

class StompCollisionSpace
{

public:
  StompCollisionSpace(ros::NodeHandle node_handle);
  virtual ~StompCollisionSpace();

  /**
   * \brief Initializes the collision space, listens for messages, etc
   *
   * \return false if not successful
   */
  bool init(double max_radius_clearance, std::string& reference_frame);

  double getDistanceGradient(double x, double y, double z,
      double& gradient_x, double& gradient_y, double& gradient_z) const;

  double getDistance(double x, double y, double z) const;

  void setPlanningScene(const arm_navigation_msgs::PlanningScene& planning_scene);

  const arm_navigation_msgs::PlanningScene& getPlanningScene();
  //void setStartState(const StompRobotModel::StompPlanningGroup& planning_group, const arm_navigation_msgs::RobotState& robot_state);

  inline void worldToGrid(tf::Vector3 origin, double wx, double wy, double wz, int &gx, int &gy, int &gz) const;

  inline void gridToWorld(tf::Vector3 origin, int gx, int gy, int gz, double &wx, double &wy, double &wz) const;

  bool getCollisionPointPotentialGradient(const StompCollisionPoint& collision_point, const KDL::Vector& collision_point_pos,
      double& potential, bool compute_gradient, Eigen::Vector3d& gradient) const;
  bool getCollisionPointPotential(const StompCollisionPoint& collision_point, const KDL::Vector& collision_point_pos, double& potential) const;
  bool getCollisionPointDistance(const StompCollisionPoint& collision_point, const KDL::Vector& collision_point_pos, double& distance) const;

private:

  std::vector<tf::Vector3> interpolateTriangle(tf::Vector3 v0, 
                                             tf::Vector3 v1, 
                                             tf::Vector3 v2, double min_res);
 
  double dist(const tf::Vector3 &v0, const tf::Vector3 &v1)
  {
    return sqrt( (v1.x()-v0.x())*(v1.x()-v0.x()) + 
                 (v1.y()-v0.y())*(v1.y()-v0.y()) +  
                 (v1.z()-v0.z())*(v1.z()-v0.z()) );
  }
 
  ros::NodeHandle node_handle_, root_handle_;

  ros::Publisher viz_pub_;
  boost::shared_ptr<distance_field::SignedPropagationDistanceField> distance_field_;

  std::string reference_frame_;

  double max_expansion_;
  double resolution_;

  arm_navigation_msgs::PlanningScene planning_scene_;

  void getVoxelsInBody(const bodies::Body &body, std::vector<tf::Vector3> &voxels);
  void addCollisionObjectToPoints(std::vector<tf::Vector3>& points, const arm_navigation_msgs::CollisionObject& object);

};

///////////////////////////// inline functions follow ///////////////////////////////////

inline double StompCollisionSpace::getDistanceGradient(double x, double y, double z,
    double& gradient_x, double& gradient_y, double& gradient_z) const
{
  return distance_field_->getDistanceGradient(x, y, z, gradient_x, gradient_y, gradient_z);
}

inline double StompCollisionSpace::getDistance(double x, double y, double z) const
{
  return distance_field_->getDistance(x,y,z);
  //return distance_field_->get
}

inline bool StompCollisionSpace::getCollisionPointDistance(const StompCollisionPoint& collision_point, const KDL::Vector& collision_point_pos, double& distance) const
{
  distance = getDistance(collision_point_pos.x(), collision_point_pos.y(), collision_point_pos.z());
  distance -= collision_point.getRadius();
  return distance <= 0.0;
}

inline bool StompCollisionSpace::getCollisionPointPotentialGradient(const StompCollisionPoint& collision_point, const KDL::Vector& collision_point_pos,
    double& potential, bool compute_gradient, Eigen::Vector3d& gradient) const
{
  Eigen::Vector3d field_gradient;
  double field_distance;

  if (compute_gradient)
  {
    field_distance = getDistanceGradient(
      collision_point_pos.x(), collision_point_pos.y(), collision_point_pos.z(),
      field_gradient(0), field_gradient(1), field_gradient(2));
  }
  else
  {
    field_distance = getDistance(collision_point_pos.x(), collision_point_pos.y(), collision_point_pos.z());
    field_gradient.setZero(3);
  }

  double d = field_distance - collision_point.getRadius();

  // three cases below:
  if (d >= collision_point.getClearance())
  {
    potential = 0.0;
    gradient.setZero(3);
  }
  else if (d >= 0.0)
  {
    double diff = (d - collision_point.getClearance());
    double gradient_magnitude = diff * collision_point.getInvClearance(); // (diff / clearance)
    potential = 0.5*gradient_magnitude*diff;
    gradient = gradient_magnitude * field_gradient;
  }
  else // if d < 0.0
  {
    gradient = field_gradient;
    potential = -d + 0.5 * collision_point.getClearance();
  }

  return (field_distance <= collision_point.getRadius()); // true if point is in collision
}

inline bool StompCollisionSpace::getCollisionPointPotential(const StompCollisionPoint& collision_point, const KDL::Vector& collision_point_pos, double& potential) const
{
  Eigen::Vector3d gradient;
  return getCollisionPointPotentialGradient(collision_point, collision_point_pos, potential, false, gradient);
}

inline void StompCollisionSpace::worldToGrid(tf::Vector3 origin, double wx, double wy, double wz, int &gx, int &gy, int &gz) const {
  gx = (int)((wx - origin.x()) * (1.0 / resolution_));
  gy = (int)((wy - origin.y()) * (1.0 / resolution_));
  gz = (int)((wz - origin.z()) * (1.0 / resolution_));
}

/** \brief Convert from voxel grid coordinates to world coordinates. */
inline void StompCollisionSpace::gridToWorld(tf::Vector3 origin, int gx, int gy, int gz, double &wx, double &wy, double &wz) const {
  wx = gx * resolution_ + origin.x();
  wy = gy * resolution_ + origin.y();
  wz = gz * resolution_ + origin.z();
}

} // namespace stomp

#endif /* STOMP_COLLISION_SPACE_H_ */
