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

#ifndef STOMP_COLLISION_POINT_H_
#define STOMP_COLLISION_POINT_H_

#include <kdl/frames.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

namespace stomp_ros_interface
{

class StompCollisionPoint
{
public:
  StompCollisionPoint(const std::vector<int>& parent_joints, double radius, double clearance,
      int segment_number, const KDL::Vector& position);
  StompCollisionPoint(const StompCollisionPoint &point, const std::vector<int>& parent_joints);
  virtual ~StompCollisionPoint();

  bool isParentJoint(int joint) const;
  double getRadius() const;
  double getVolume() const;
  double getClearance() const;
  double getInvClearance() const;
  int getSegmentNumber() const;
  const KDL::Vector& getPosition() const;

  const std::vector<int>& getParentJoints() const {
    return parent_joints_;
  }

  void getTransformedPosition(std::vector<KDL::Frame>& segment_frames, KDL::Vector& position) const;

  template<typename Derived>
  void getJacobian(std::vector<Eigen::Map<Eigen::Vector3d> >& joint_pos, std::vector<Eigen::Map<Eigen::Vector3d> >& joint_axis,
      Eigen::Map<Eigen::Vector3d>& collision_point_pos, Eigen::MatrixBase<Derived>& jacobian, const std::vector<int>& group_joint_to_kdl_joint_index) const;

private:
  std::vector<int> parent_joints_;      /**< Which joints can influence the motion of this point */
  double radius_;                       /**< Radius of the sphere */
  double volume_;                       /**< Volume of the sphere */
  double clearance_;                    /**< Extra clearance required while optimizing */
  double inv_clearance_;                /**< 1/clearance_ pre-computed */
  int segment_number_;                  /**< Which segment does this point belong to */
  KDL::Vector position_;                /**< Vector of this point in the frame of the above segment */
};

inline bool StompCollisionPoint::isParentJoint(int joint) const
{
  return(find(parent_joints_.begin(), parent_joints_.end(), joint) != parent_joints_.end());
}

inline double StompCollisionPoint::getRadius() const
{
  return radius_;
}

inline double StompCollisionPoint::getVolume() const
{
  return volume_;
}

inline double StompCollisionPoint::getClearance() const
{
  return clearance_;
}

inline double StompCollisionPoint::getInvClearance() const
{
  return inv_clearance_;
}

inline int StompCollisionPoint::getSegmentNumber() const
{
  return segment_number_;
}

inline const KDL::Vector& StompCollisionPoint::getPosition() const
{
  return position_;
}

template<typename Derived>
void StompCollisionPoint::getJacobian(std::vector<Eigen::Map<Eigen::Vector3d> >& joint_pos, std::vector<Eigen::Map<Eigen::Vector3d> >& joint_axis,
    Eigen::Map<Eigen::Vector3d>& collision_point_pos, Eigen::MatrixBase<Derived>& jacobian, const std::vector<int>& group_joint_to_kdl_joint_index) const
{

  for(unsigned int joint = 0; joint < group_joint_to_kdl_joint_index.size(); joint++) {
    if(!isParentJoint(group_joint_to_kdl_joint_index[joint])) {
      // since the joint is not active, fill the jacobian column with zeros
      jacobian.col(joint).setZero();
    }
    else
    {
      int kj = group_joint_to_kdl_joint_index[joint];
      jacobian.col(joint) = joint_axis[kj].cross(collision_point_pos - joint_pos[kj]);
    }
  }
}

inline void StompCollisionPoint::getTransformedPosition(std::vector<KDL::Frame>& segment_frames, KDL::Vector& position) const
{
  position = segment_frames[segment_number_] * position_;
}

} // namespace stomp

#endif /* STOMP_COLLISION_POINT_H_ */
