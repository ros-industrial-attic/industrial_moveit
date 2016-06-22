/**
 * @file constrained_ik_planner_plugin.h
 * @brief This class represents the CLIK planner plugin for moveit.  It manages all of the CLIK planners.
 *
 * @author Levi Armstrong
 * @date May 4, 2015
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2015, Southwest Research Institute
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
#ifndef CONSTRAINED_IK_PLANNER_PLUGIN_H
#define CONSTRAINED_IK_PLANNER_PLUGIN_H
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <class_loader/class_loader.h>
#include <ros/ros.h>
#include <constrained_ik/moveit_interface/joint_interpolation_planner.h>
#include <constrained_ik/moveit_interface/cartesian_planner.h>
#include <constrained_ik/moveit_interface/constrained_ik_planner_parameters.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <constrained_ik/ConstrainedIKPlannerDynamicReconfigureConfig.h>

namespace constrained_ik
{
  /**
   * @brief This class represents the CLIK planner plugin for moveit.  It
   * manages all of the CLIK planners.
   */
  class CLIKPlannerManager : public planning_interface::PlannerManager
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CLIKPlannerManager() : planning_interface::PlannerManager(),
                           nh_("~")
    {
    }

    bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns);

    bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const {return req.trajectory_constraints.constraints.empty();}

    std::string getDescription() const { return "CLIK"; }

    void getPlanningAlgorithms(std::vector<std::string> &algs) const;

    void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs);

    planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene, const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const;

    void dynamicReconfigureCallback(ConstrainedIKPlannerDynamicReconfigureConfig &config, uint32_t level);

  protected:
    typedef dynamic_reconfigure::Server<ConstrainedIKPlannerDynamicReconfigureConfig> DynReconfigServer;

    ros::NodeHandle nh_;
    ConstrainedIKPlannerDynamicReconfigureConfig config_;
    boost::scoped_ptr<DynReconfigServer> dynamic_reconfigure_server_;
    boost::recursive_mutex mutex_;

    /** Containes all the availble CLIK planners */
    std::map< std::string, constrained_ik::CLIKPlanningContextPtr> planners_;
  };
} //namespace constrained_ik
#endif //CONSTRAINED_IK_PLANNER_PLUGIN_H
