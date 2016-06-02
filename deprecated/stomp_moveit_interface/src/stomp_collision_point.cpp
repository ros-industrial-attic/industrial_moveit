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

#include <stomp_ros_interface/stomp_collision_point.h>
#include <cmath>


namespace stomp_ros_interface
{

StompCollisionPoint::StompCollisionPoint(const std::vector<int>& parent_joints, double radius, double clearance,
    int segment_number, const KDL::Vector& position):
      parent_joints_(parent_joints),
      radius_(radius),
      volume_((4.0/3.0)*M_PI*radius_*radius_*radius_),
      clearance_(clearance),
      inv_clearance_(1.0/clearance_),
      segment_number_(segment_number),
      position_(position)
{

}

StompCollisionPoint::StompCollisionPoint(const StompCollisionPoint &point, const std::vector<int>& parent_joints):
  parent_joints_(parent_joints),
  radius_(point.radius_),
  volume_((4.0/3.0)*M_PI*radius_*radius_*radius_),
  clearance_(point.clearance_),
  inv_clearance_(1.0/clearance_),
  segment_number_(point.segment_number_),
  position_(point.position_)
{
}

StompCollisionPoint::~StompCollisionPoint()
{
}

}
