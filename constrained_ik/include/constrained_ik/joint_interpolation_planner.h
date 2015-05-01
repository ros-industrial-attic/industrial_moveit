/*
 * joint_interpolation_planner.h
 *
 *  Created on: May 4, 2015
 *      Author: Levi Armstrong
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
#ifndef JOINT_INTERPOLATION_PLANNER_H
#define JOINT_INTERPOLATION_PLANNER_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <constrained_ik/constrained_ik_planning_context.h>

namespace constrained_ik
{
/**
   * @brief The JointInterpolationPlanner class
   *
   * This class is used to represent a joint interpolated path planner for
   * moveit.  This planner does not have the inherent ability to avoid
   * collision. It does check if the path created is collision free before it
   * returns a trajectory.  If a collision is found it returns an empty
   * trajectory and moveit error.
   */
  class JointInterpolationPlanner : public constrained_ik::CLIKPlanningContext
  {
  public:
    JointInterpolationPlanner(const std::string &name, const std::string &group) : constrained_ik::CLIKPlanningContext(name, group) {}

    void clear() { params_.reset(); }

    bool terminate() {}

    bool solve(planning_interface::MotionPlanResponse &res);

  };
} //namespace constrained_ik

#endif // JOINT_INTERPOLATION_PLANNER_H
