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

#include <stomp_ros_interface/treefksolverjointposaxis_partial.hpp>
#include <iostream>

using namespace std;

namespace KDL {

TreeFkSolverJointPosAxisPartial::TreeFkSolverJointPosAxisPartial(const Tree& tree, const std::string& reference_frame, const std::vector<bool>& active_joints):
  tree_(tree),
  reference_frame_(reference_frame),
  active_joints_(active_joints)
{
  segment_names_.clear();
  assignSegmentNumber(tree_.getRootSegment());
  std::map<std::string, int>::iterator reference_frame_it = segment_name_to_index_.find(reference_frame);
  if (reference_frame_it == segment_name_to_index_.end())
  {
    cout << "TreeFkSolverJointPosAxisPartial: Reference frame " << reference_frame << " could not be found! Forward kinematics will be performed in world frame.";
  }
  else
  {
    reference_frame_index_ = reference_frame_it->second;
  }
  num_segments_ = segment_names_.size();
  num_joints_ = tree_.getNrOfJoints();
  segment_frames_.resize(num_segments_);
  segment_parent_frame_nr_.resize(num_segments_);
  segment_parent_.resize(num_segments_, NULL);
  joint_parent_frame_nr_.resize(num_joints_);
  joint_parent_.resize(num_joints_, NULL);
  segment_evaluation_order_.clear();
  joint_calc_pos_axis_.clear();
  joint_calc_pos_axis_.resize(num_joints_, false);
  full_fk_done_ = false;
}

TreeFkSolverJointPosAxisPartial::~TreeFkSolverJointPosAxisPartial()
{
}

//int TreeFkSolverJointPosAxisPartial::JntToCart(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames)
//{
//  if (!full_fk_done_)
//  {
//    full_fk_done_ = true;
//    return JntToCartFull(q_in, joint_pos, joint_axis, segment_frames);
//  }
//  else
//  {
//    return JntToCartPartial(q_in, joint_pos, joint_axis, segment_frames);
//  }
//}
//
//void TreeFkSolverJointPosAxisPartial::resetState()
//{
//  full_fk_done_ = false;
//}

int TreeFkSolverJointPosAxisPartial::JntToCartFull(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames)
{
  joint_pos.resize(num_joints_);
  joint_axis.resize(num_joints_);
  segment_frames.resize(num_segments_);

  segment_evaluation_order_.clear();

  // start the recursion
  treeRecursiveFK(q_in, joint_pos, joint_axis, segment_frames, Frame::Identity(), tree_.getRootSegment(), 0, -1, false);

  // get the inverse reference frame:
  Frame inv_ref_frame = segment_frames[reference_frame_index_].Inverse();

  // convert all the frames into the reference frame:
  for (int i=0; i<num_segments_; i++)
  {
    segment_frames[i] = inv_ref_frame * segment_frames[i];
  }

  // convert all joint positions and axes into reference frame:
  for (int i=0; i<num_joints_; i++)
  {
    joint_axis[i] = inv_ref_frame * joint_axis[i];
    joint_pos[i] = inv_ref_frame * joint_pos[i];
  }

  segment_frames_ = segment_frames;

  return 0;
}

int TreeFkSolverJointPosAxisPartial::JntToCartPartial(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames) const
{
  joint_pos.resize(num_joints_);
  joint_axis.resize(num_joints_);
  segment_frames.resize(num_segments_);

  // first solve for all segments
  for (size_t i=0; i<segment_evaluation_order_.size(); ++i)
  {
    int segment_nr = segment_evaluation_order_[i];
    const TreeElement* parent_segment = segment_parent_[segment_nr];
    double jnt_p = 0;
    if (parent_segment->segment.getJoint().getType() != Joint::None)
    {
      jnt_p = q_in(parent_segment->q_nr);
    }

    segment_frames[segment_nr] =  segment_frames[segment_parent_frame_nr_[segment_nr]] * parent_segment->segment.pose(jnt_p);
  }

  // now solve for joint positions and axes:
  for (int i=0; i<num_joints_; ++i)
  {
    if (joint_calc_pos_axis_[i])
    {
      Frame& frame = segment_frames[joint_parent_frame_nr_[i]];
      const TreeElement* parent_segment = joint_parent_[i];
      joint_pos[i] = frame * parent_segment->segment.getJoint().JointOrigin();
      joint_axis[i] = frame.M * parent_segment->segment.getJoint().JointAxis();
    }
  }
  return 0;
}

int TreeFkSolverJointPosAxisPartial::treeRecursiveFK(const JntArray& q_in, std::vector<Vector>& joint_pos, std::vector<Vector>& joint_axis, std::vector<Frame>& segment_frames,
    const Frame& previous_frame, const SegmentMap::const_iterator this_segment, int segment_nr, int parent_segment_nr, bool active)
{
  Frame this_frame = previous_frame;

  // get the joint angle:
  double jnt_p = 0;
  if (this_segment->second.segment.getJoint().getType() != Joint::None)
  {
    int q_nr = this_segment->second.q_nr;
    jnt_p = q_in(q_nr);
    joint_parent_frame_nr_[q_nr] = parent_segment_nr;
    joint_parent_[q_nr] = &(this_segment->second);
    joint_pos[q_nr] = this_frame * this_segment->second.segment.getJoint().JointOrigin();
    joint_axis[q_nr] = this_frame.M * this_segment->second.segment.getJoint().JointAxis();
    if (active && active_joints_[q_nr])
      joint_calc_pos_axis_[q_nr] = true;
    if (active_joints_[q_nr])
      active = true;
  }

  // do the FK:
  if (active)
    segment_evaluation_order_.push_back(segment_nr);
  segment_parent_frame_nr_[segment_nr] = parent_segment_nr;
  segment_parent_[segment_nr] = &(this_segment->second);
  this_frame = this_frame * this_segment->second.segment.pose(jnt_p);
  segment_frames[segment_nr] = this_frame;

  int par_seg_nr = segment_nr;
  segment_nr++;

  // get poses of child segments
  for (vector<SegmentMap::const_iterator>::const_iterator child=this_segment->second.children.begin(); child !=this_segment->second.children.end(); child++)
    segment_nr = treeRecursiveFK(q_in, joint_pos, joint_axis, segment_frames, this_frame, *child, segment_nr, par_seg_nr, active);
  return segment_nr;
}

void TreeFkSolverJointPosAxisPartial::assignSegmentNumber(const SegmentMap::const_iterator this_segment)
{
  int num = segment_names_.size();
  segment_names_.push_back(this_segment->first);
  segment_name_to_index_[this_segment->first] = num;

  // add the child segments recursively
  for (vector<SegmentMap::const_iterator>::const_iterator child=this_segment->second.children.begin(); child !=this_segment->second.children.end(); child++)
  {
    assignSegmentNumber(*child);
  }
}

const std::vector<std::string> TreeFkSolverJointPosAxisPartial::getSegmentNames() const
{
  return segment_names_;
}

const std::map<std::string, int> TreeFkSolverJointPosAxisPartial::getSegmentNameToIndex() const
{
  return segment_name_to_index_;
}

int TreeFkSolverJointPosAxisPartial::segmentNameToIndex(std::string name) const
{
  return segment_name_to_index_.find(name)->second;
}

} // namespace KDL

