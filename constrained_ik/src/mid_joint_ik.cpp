/*
 * mid_joint_ik.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: dsolomon
 */
/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <constrained_ik/mid_joint_ik.h>
#include <ros/ros.h>

namespace constrained_ik
{

namespace mid_joint_ik
{

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Affine3d;
using basic_ik::Basic_IK;

MidJoint_IK::MidJoint_IK() : Basic_IK()
{
	rot_err_ = M_PI;
}

void MidJoint_IK::augmentErr(VectorXd &err)
{
	const double weight_mj = .1;
	const size_t orig_size(err.size()), n(kin_.numJoints());
	err.conservativeResize(orig_size + n);
	for (size_t ii=0; ii<n; ++ii)
	{
		err(ii + orig_size) = weight_mj * (mid_joint_(ii) - joints_(ii));
	}
}

void MidJoint_IK::augmentJ(MatrixXd &J)
{
	const size_t n(kin_.numJoints()), orig_rows(J.rows()), orig_cols(J.cols());
	J.conservativeResize(orig_rows+n, orig_cols);
	J.block(orig_rows, 0, n, orig_cols) = MatrixXd::Identity(n, orig_cols);
}

void MidJoint_IK::calcInvKin(const Affine3d &goal, const VectorXd &joint_seed, VectorXd &joint_angles)
{
	if (!checkInitialized())
		throw std::runtime_error("Must call init() before using Constrained_IK");

	// initialize state
	joint_angles = joint_seed;  // initialize result to seed value
	reset(goal, joint_seed);    // reset state vars for this IK solve
	update(joint_angles);       // update current state

	// iterate until solution converges (or aborted)
	while (!checkStatus())
	{
		// calculate a Jacobian (relating joint-space updates/deltas to cartesian-space errors/deltas)
		// and the associated cartesian-space error/delta vector
		MatrixXd J  = calcConstraintJacobian();
		VectorXd err = calcConstraintError();

		// solve for the resulting joint-space update
		VectorXd dJoint;


		augmentJ(J);	// add augmented rows to J to push joints back to mid position
		augmentErr(err);
		kin_.solvePInv(J, err, dJoint);

		// update joint solution by the calculated update (or a partial fraction)
		joint_angles += dJoint * joint_update_gain_;
		clipToJointLimits(joint_angles);

		// re-update internal state variables
		update(joint_angles);
	}

	ROS_INFO_STREAM("IK solution: " << joint_angles.transpose());
}

void MidJoint_IK::init(const basic_kin::BasicKin &kin)
{
	if (!kin.checkInitialized())
		throw std::invalid_argument("Input argument 'BasicKin' must be initialized");

	kin_ = kin;

	const MatrixXd limits = kin.getLimits();
	const size_t n(limits.rows());
	mid_joint_.resize(n);
	for (size_t ii=0; ii<n; ++ii)
	{
		mid_joint_(ii) = (limits(ii,0) + limits(ii,1)) / 2.0;
	}

	initialized_ = true;
}

} /* namespace mid_joint_ik */
} /* namespace constrained_ik */
