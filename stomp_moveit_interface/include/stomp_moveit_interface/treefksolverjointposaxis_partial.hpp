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

/** \author Wim Meeussen, Mrinal Kalakrishnan */

#ifndef KDLTREEFKSOLVERJOINTPOSAXIS_PARTIAL_HPP
#define KDLTREEFKSOLVERJOINTPOSAXIS_PARTIAL_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <vector>

namespace KDL {

/**
 * \brief A solver which can perform forward kinematics for a limited set of joints
 * Returns the joint positions, joint axes and segment frames
 */
class TreeFkSolverJointPosAxisPartial

{
public:
  TreeFkSolverJointPosAxisPartial(const Tree& tree, const std::string& reference_frame, const std::vector<bool>& active_joints);
  ~TreeFkSolverJointPosAxisPartial();

//  int JntToCart(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames);
//  void resetState();
  int JntToCartFull(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames);
  int JntToCartPartial(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames) const;

  const std::vector<std::string> getSegmentNames() const;
  const std::map<std::string, int> getSegmentNameToIndex() const;

  int segmentNameToIndex(std::string name) const;

private:

  int treeRecursiveFK(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames,
      const Frame& previous_frame, const SegmentMap::const_iterator this_segment, int segment_nr, int parent_segment_nr, bool active);

  bool full_fk_done_;

  std::vector<std::string> segment_names_;
  std::map<std::string, int> segment_name_to_index_;
  Tree tree_;
  std::string reference_frame_;
  int reference_frame_index_;
  int num_joints_;
  int num_segments_;

  std::vector<Frame> segment_frames_;           /**< local cache of all segment frames needed for partial FK */
  std::vector<int> segment_evaluation_order_;   /**< what order should we evaluate segments in */
  std::vector<int> segment_parent_frame_nr_;            /**< the parent frame number for each segment */
  std::vector<const TreeElement*> segment_parent_;            /**< the parent segment for each segment */
  std::vector<int> joint_parent_frame_nr_;            /**< the parent frame number for each joint */
  std::vector<const TreeElement*> joint_parent_;            /**< the parent segment for each joint */
  std::vector<bool> active_joints_;             /**< which are the joints that will change in calls to partial FK */
  std::vector<bool> joint_calc_pos_axis_;       /**< which joints should we calculate the position and axis for */

  void assignSegmentNumber(const SegmentMap::const_iterator this_segment);

};

} // namespace KDL

#endif
