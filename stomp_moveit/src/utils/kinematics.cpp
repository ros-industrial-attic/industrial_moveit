/**
 * @file kinematics.cpp
 * @brief This defines kinematic related utilities.
 *
 * @author Jorge Nicho
 * @date June 26, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stomp_moveit/utils/kinematics.h>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <math.h>

static const std::string DEBUG_NS = "stomp_moveit_kinematics";

KDL::JntArray toKDLJntArray(const std::vector<double>& vals)
{
  KDL::JntArray jarr(vals.size());
  for(int i = 0; i < jarr.rows(); i++)
  {
    jarr(i) = vals[i];
  }
  return std::move(jarr);
}

KDL::Chain createKDLChain(moveit::core::RobotModelConstPtr robot_model,std::string base_link,std::string tip_link)
{
  KDL::Tree tree;
  KDL::Chain chain;
  kdl_parser::treeFromUrdfModel(*robot_model->getURDF(),tree);
  tree.getChain(base_link,tip_link,chain);
  return chain;
}

std::shared_ptr<TRAC_IK::TRAC_IK> createTRACIKSolver(moveit::core::RobotModelConstPtr robot_model,const moveit::core::JointModelGroup* group,double max_time = 0.005)
{
  using namespace moveit::core;

  std::string base_link = group->getActiveJointModels().front()->getParentLinkModel()->getName();
  std::string tip_link = group->getLinkModelNames().back();
  int num_joints = group->getActiveJointModelNames().size();

  ROS_DEBUG_NAMED(DEBUG_NS,"Creating IK solver with base link '%s' and tip link '%s'",base_link.c_str(),tip_link.c_str());

  // create chain
  KDL::Chain kdl_chain = createKDLChain(robot_model,base_link,tip_link);

  // create joint array limits
  const JointBoundsVector &bounds = group->getActiveJointModelsBounds();
  std::vector<double> min_vals, max_vals;
  for(int i = 0; i < num_joints;i++)
  {
    const JointModel::Bounds *jb = bounds[i];
    for(auto& b: *jb)
    {
      max_vals.push_back(b.max_position_);
      min_vals.push_back(b.min_position_);
    }
  }

  // copying to KDL joint arrays
  num_joints = min_vals.size();
  KDL::JntArray jmin = toKDLJntArray(min_vals);
  KDL::JntArray jmax = toKDLJntArray(max_vals);
  std::shared_ptr<TRAC_IK::TRAC_IK> solver(new TRAC_IK::TRAC_IK(kdl_chain,jmin,jmax,max_time));
  return solver;
}

namespace stomp_moveit
{
namespace utils
{

/**
 * @namespace stomp_moveit::utils::kinematics
 * @brief Utility functions related to finding Inverse Kinematics solutions
 */
namespace kinematics
{

  IKSolver::IKSolver(moveit::core::RobotModelConstPtr robot_model,std::string group_name,double max_time)
  {
    const moveit::core::JointModelGroup* group = robot_model->getJointModelGroup(group_name);
    ik_solver_impl_ = createTRACIKSolver(robot_model,group,max_time);
  }

  IKSolver::~IKSolver()
  {

  }

  bool IKSolver::solve(const Eigen::VectorXd& seed,const Eigen::Affine3d& tool_pose,Eigen::VectorXd& solution,
             Eigen::VectorXd tol)
  {
    using namespace Eigen;
    std::vector<double> seed_(seed.size(),0);
    std::vector<double> solution_;
    std::vector<double> tol_(6,0);

    // converting to eigen
    VectorXd::Map(&seed_.front(),seed.size()) = seed;
    VectorXd::Map(&tol_.front(),tol_.size()) = tol;

    if(!solve(seed_,tool_pose,solution_,tol_))
    {
      return false;
    }

    solution.resize(solution_.size());
    solution = VectorXd::Map(solution_.data(),solution.size());
    return true;
  }

  bool IKSolver::solve(const std::vector<double>& seed, const Eigen::Affine3d& tool_pose,std::vector<double>& solution,
             std::vector<double> tol)
  {
    using namespace KDL;
    using namespace Eigen;
    JntArray seed_kdl = toKDLJntArray(seed);
    Twist tol_kdl;
    JntArray solution_kdl;
    Frame tool_pose_kdl;

    // converting to KDL data types
    for(int i = 0; i < tol.size(); i++)
    {
      tol_kdl[i] = tol[i];
    }

    tf::transformEigenToKDL(tool_pose, tool_pose_kdl);


    // calling solver
    if(ik_solver_impl_->CartToJnt(seed_kdl,tool_pose_kdl,solution_kdl,tol_kdl) <= 0)
    {
      ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"Failed to solve IK for tool pose:\n"<<tool_pose.matrix());
      ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"Cartesian tolerance used :\n"<<tol_kdl[0]<<" "<<tol_kdl[1]<<" "<<tol_kdl[2]<<" "<<tol_kdl[3]<<" "<<tol_kdl[4]<<" "<<tol_kdl[5]);
      return false;
    }

    solution.resize(solution_kdl.rows());
    VectorXd::Map(&solution.front(),solution.size()) = solution_kdl.data;
    return true;
  }

  bool IKSolver::solve(const std::vector<double>& seed, const moveit_msgs::Constraints& tool_constraints,std::vector<double>& solution)
  {
    Eigen::Affine3d tool_pose;
    std::vector<double> tolerance;
    if(!decodeCartesianConstraint(tool_constraints,tool_pose,tolerance))
    {
      return false;
    }

    return solve(seed,tool_pose,solution,tolerance);
  }

  bool IKSolver::solve(const Eigen::VectorXd& seed, const moveit_msgs::Constraints& tool_constraints,Eigen::VectorXd& solution)
  {
    Eigen::Affine3d tool_pose;
    std::vector<double> tolerance;
    if(!decodeCartesianConstraint(tool_constraints,tool_pose,tolerance))
    {
      return false;
    }

    Eigen::VectorXd tolerance_eigen = Eigen::VectorXd::Zero(6);
    tolerance_eigen = Eigen::VectorXd::Map(tolerance.data(),tolerance.size());
    return solve(seed,tool_pose,solution,tolerance_eigen);
  }

  bool decodeCartesianConstraint(const moveit_msgs::Constraints& constraints, Eigen::Affine3d& tool_pose, Eigen::VectorXd& tolerance)
  {
    std::vector<double> tolerance_std;
    if(!decodeCartesianConstraint(constraints,tool_pose,tolerance_std))
    {
      return false;
    }

    tolerance = Eigen::VectorXd::Map(tolerance_std.data(),tolerance_std.size());
    return true;
  }

  bool decodeCartesianConstraint(const moveit_msgs::Constraints& constraints, Eigen::Affine3d& tool_pose, std::vector<double>& tolerance)
  {
    using namespace Eigen;

    // declaring result variables
    tolerance.resize(6,0);
    geometry_msgs::Point p;
    Eigen::Quaterniond q = Quaterniond::Identity();
    const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = constraints.position_constraints;
    const std::vector<moveit_msgs::OrientationConstraint>& orient_constraints = constraints.orientation_constraints;

    int num_constraints = pos_constraints.size() > orient_constraints.size() ? pos_constraints.size() : orient_constraints.size();
    if(num_constraints == 0)
    {
     ROS_ERROR("Constraints message is empty");
     return false;
    }

    // position
    if(pos_constraints.empty())
    {
     ROS_ERROR("Position constraint is empty");
     return false;
    }
    else
    {
     const moveit_msgs::PositionConstraint& pos_constraint = pos_constraints[0];

     // tool pose nominal position
     p = pos_constraint.constraint_region.primitive_poses[0].position;

     // collecting position tolerances
     const shape_msgs::SolidPrimitive& bv = pos_constraint.constraint_region.primitives[0];
     bool valid_constraint = true;
     switch(bv.type)
     {
       case shape_msgs::SolidPrimitive::BOX :
       {
         if(bv.dimensions.size() != 3)
         {
           ROS_WARN("Position constraint for BOX shape incorrectly defined, only 3 dimensions entries are needed");
           valid_constraint = false;
           break;
         }

         using SP = shape_msgs::SolidPrimitive;
         tolerance[0] = bv.dimensions[SP::BOX_X];
         tolerance[1] = bv.dimensions[SP::BOX_Y];
         tolerance[2] = bv.dimensions[SP::BOX_Z];
       }
       break;

       case shape_msgs::SolidPrimitive::SPHERE:
       {
         if(bv.dimensions.size() != 1)
         {
           ROS_WARN("Position constraint for SPHERE shape has no valid dimensions");
           valid_constraint = false;
           break;
         }

         using SP = shape_msgs::SolidPrimitive;
         tolerance[0] = bv.dimensions[SP::SPHERE_RADIUS];
         tolerance[1] = bv.dimensions[SP::SPHERE_RADIUS];
         tolerance[2] = bv.dimensions[SP::SPHERE_RADIUS];
       }
       break;

       default:

         ROS_WARN("The Position constraint shape %i isn't supported, defaulting position tolerance to zero",bv.type);
         break;
     }
    }

    // orientation
    if(orient_constraints.size() == 0)
    {
     // setting tolerance to zero
     std::fill(tolerance.begin()+3,tolerance.end(),M_PI);
    }
    else
    {
     const moveit_msgs::OrientationConstraint& orient_constraint = orient_constraints[0];

     // collecting orientation tolerance
     tolerance[3] = orient_constraint.absolute_x_axis_tolerance;
     tolerance[4] = orient_constraint.absolute_y_axis_tolerance;
     tolerance[5] = orient_constraint.absolute_z_axis_tolerance;

     // tool pose nominal orientation
     tf::quaternionMsgToEigen(orient_constraint.orientation,q);
    }

    // assembling tool pose
    tool_pose = Affine3d::Identity()*Eigen::Translation3d(Eigen::Vector3d(p.x,p.y,p.z))*q;

    return true;
  }

  std::vector<Eigen::Affine3d> sampleCartesianPoses(const moveit_msgs::Constraints& c,const std::vector<double> sampling_resolution,int max_samples)
  {
    using namespace Eigen;

    std::vector<Eigen::Affine3d> poses;

    // random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    auto sample_val_func = [&gen](double min, double max, double intrv) -> double
    {
      double length = max - min;
      if(length <= intrv || length < 1e-6)
      {
        return 0.5*(max + min); // return average
      }

      int num_intervals = std::ceil(length/intrv);
      std::uniform_int_distribution<> dis(0, num_intervals);
      int r = dis(gen);
      double val = min + r*(intrv);
      val = val > max ? max : val;
      return val;
    };

    // extracting tolerances from constraints
    const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = c.position_constraints;
    const std::vector<moveit_msgs::OrientationConstraint>& orient_constraints = c.orientation_constraints;
    for(std::size_t k = 0; k < pos_constraints.size(); k++)
    {

      const moveit_msgs::PositionConstraint& pos_constraint = pos_constraints[k];
      const moveit_msgs::OrientationConstraint& orient_constraint = orient_constraints[k];

      // collecting position tolerances
      std::vector<double> tolerances(6);
      const shape_msgs::SolidPrimitive& bv = pos_constraint.constraint_region.primitives[0];
      bool valid_constraint = true;
      switch(bv.type)
      {
        case shape_msgs::SolidPrimitive::BOX :
        {
          if(bv.dimensions.size() != 3)
          {
            ROS_WARN("Position constraint for BOX shape incorrectly defined, only 3 dimensions entries are needed");
            valid_constraint = false;
            break;
          }

          using SP = shape_msgs::SolidPrimitive;
          tolerances[0] = bv.dimensions[SP::BOX_X];
          tolerances[1] = bv.dimensions[SP::BOX_Y];
          tolerances[2] = bv.dimensions[SP::BOX_Z];
        }
        break;

        case shape_msgs::SolidPrimitive::SPHERE:
        {
          if(bv.dimensions.size() != 1)
          {
            ROS_WARN("Position constraint for SPHERE shape has no valid dimensions");
            valid_constraint = false;
            break;
          }

          using SP = shape_msgs::SolidPrimitive;
          tolerances[0] = bv.dimensions[SP::SPHERE_RADIUS];
          tolerances[1] = bv.dimensions[SP::SPHERE_RADIUS];
          tolerances[2] = bv.dimensions[SP::SPHERE_RADIUS];
        }
        break;

        default:

          ROS_ERROR("The Position constraint shape %i isn't supported",bv.type);
          valid_constraint = false;
          break;
      }

      if(!valid_constraint)
      {
        continue;
      }

      // collecting orientation tolerance
      tolerances[3] = orient_constraint.absolute_x_axis_tolerance;
      tolerances[4] = orient_constraint.absolute_y_axis_tolerance;
      tolerances[5] = orient_constraint.absolute_z_axis_tolerance;

      // calculating total number of samples
      int total_num_samples = 1;
      for(std::size_t i = 0; i < tolerances.size(); i++)
      {
        total_num_samples *= (std::floor((2*tolerances[i]/sampling_resolution[i]) + 1) );
      }
      total_num_samples = total_num_samples > max_samples ? max_samples : total_num_samples;


      // tool pose nominal position
      auto& p = pos_constraint.constraint_region.primitive_poses[0].position;

      // tool pose nominal orientation
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(orient_constraint.orientation,q);

      // generating samples
      std::vector<double> vals(6);
      Affine3d nominal_pos = Affine3d::Identity()*Eigen::Translation3d(Eigen::Vector3d(p.x,p.y,p.z));
      Affine3d nominal_rot = Affine3d::Identity()*q;
      for(int i = 0; i < total_num_samples; i++)
      {
        for(int j = 0; j < vals.size() ; j++)
        {
          vals[j] = sample_val_func(-0.5*tolerances[j],0.5*tolerances[j],sampling_resolution[j]);
        }

        Affine3d rot = nominal_rot * AngleAxisd(vals[3],Vector3d::UnitX()) * AngleAxisd(vals[4],Vector3d::UnitY())
            * AngleAxisd(vals[5],Vector3d::UnitZ());
        Affine3d pose = nominal_pos * Translation3d(Vector3d(vals[0],vals[1],vals[2])) * rot;
        poses.push_back(pose);
      }

      if(poses.size()>= max_samples)
      {
        break;
      }
    }

    return poses;
  }

} // kinematics
} // utils
} // stomp_moveit




