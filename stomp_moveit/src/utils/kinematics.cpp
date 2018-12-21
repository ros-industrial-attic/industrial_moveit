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
#include <random>

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

std::shared_ptr<TRAC_IK::TRAC_IK> createTRACIKSolver(moveit::core::RobotModelConstPtr robot_model,const moveit::core::JointModelGroup* group,double max_time = 0.01)
{
  using namespace moveit::core;

  std::string base_link = group->getJointModels().front()->getParentLinkModel()->getName();
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

IKSolver::IKSolver(const moveit::core::RobotState& robot_state,std::string group_name,double max_time):
    robot_model_(robot_state.getRobotModel()),
    group_name_(group_name),
    robot_state_(new moveit::core::RobotState(robot_state))
{
  setKinematicState(robot_state);

  // create solver implementation
  ik_solver_impl_ = createTRACIKSolver(robot_model_,robot_model_->getJointModelGroup(group_name),max_time);
}

IKSolver::~IKSolver()
{

}

void IKSolver::setKinematicState(const moveit::core::RobotState& state)
{
  (*robot_state_) = state;
  robot_state_->update();

  // update transform from base to root
  const moveit::core::JointModelGroup* group = robot_state_->getJointModelGroup(group_name_);
  std::string base_link = group->getJointModels().front()->getParentLinkModel()->getName();
  Eigen::Affine3d root_to_base_tf = robot_state_->getFrameTransform(base_link);
  tf_base_to_root_ = root_to_base_tf.inverse();
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
  solution = VectorXd::Map(solution_.data(),solution_.size());
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

  // converting tolerance to KDL data types
  for(int i = 0; i < tol.size(); i++)
  {
    tol_kdl[i] = tol[i];
  }

  // converting transform to kdl data type and transforming to chain base link
  tf::transformEigenToKDL(tf_base_to_root_ * tool_pose, tool_pose_kdl);

  // calling solver
  if(ik_solver_impl_->CartToJnt(seed_kdl,tool_pose_kdl,solution_kdl,tol_kdl) <= 0)
  {
    ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"Failed to solve IK for tool pose:\n"<<tool_pose.matrix());
    ROS_DEBUG_STREAM_NAMED(DEBUG_NS,"Cartesian tolerance used :\n"<<tol_kdl[0]<<" "<<tol_kdl[1]<<" "<<tol_kdl[2]<<" "<<tol_kdl[3]<<" "<<tol_kdl[4]<<" "<<tol_kdl[5]);
    return false;
  }

  solution.resize(solution_kdl.rows());
  VectorXd::Map(&solution[0],solution.size()) = solution_kdl.data;
  return true;
}

bool IKSolver::solve(const std::vector<double>& seed, const moveit_msgs::Constraints& tool_constraints,std::vector<double>& solution)
{
  Eigen::Affine3d tool_pose;
  std::vector<double> tolerance;
  if(!decodeCartesianConstraint(robot_model_,tool_constraints,tool_pose,tolerance))
  {
    return false;
  }

  return solve(seed,tool_pose,solution,tolerance);
}

bool IKSolver::solve(const Eigen::VectorXd& seed, const moveit_msgs::Constraints& tool_constraints,Eigen::VectorXd& solution)
{
  Eigen::Affine3d tool_pose;
  std::vector<double> tolerance;
  if(!decodeCartesianConstraint(robot_model_,tool_constraints,tool_pose,tolerance))
  {
    return false;
  }

  Eigen::VectorXd tolerance_eigen = Eigen::VectorXd::Zero(6);
  tolerance_eigen = Eigen::VectorXd::Map(tolerance.data(),tolerance.size());
  return solve(seed,tool_pose,solution,tolerance_eigen);
}

bool isCartesianConstraints(const moveit_msgs::Constraints& c)
{
  std::string pos_frame_id, orient_frame_id;
  bool found_cart = false;
  if(c.position_constraints.empty() && c.orientation_constraints.empty())
  {
    ROS_DEBUG("No cartesian constraints were found, failed validation");
    return false;
  }

  if(!c.position_constraints.empty())
  {
    pos_frame_id = c.position_constraints.front().header.frame_id;
    found_cart = !pos_frame_id.empty();
  }

  if(!c.orientation_constraints.empty())
  {
    orient_frame_id = c.orientation_constraints.front().header.frame_id;
    found_cart = !orient_frame_id.empty();
  }

  if(!pos_frame_id.empty() && !orient_frame_id.empty() && (pos_frame_id != orient_frame_id))
  {
    ROS_ERROR("Position frame id '%s' differs from orientation frame id '%s', invalid Cartesian constraint",
              pos_frame_id.c_str(), orient_frame_id.c_str());
    return false;
  }

  return found_cart;
}

boost::optional<moveit_msgs::Constraints> curateCartesianConstraints(const moveit_msgs::Constraints& c,const Eigen::Affine3d& ref_pose,
                                                              double default_pos_tol , double default_rot_tol)
{
  using namespace moveit_msgs;

  moveit_msgs::Constraints full_constraint;
  if(!isCartesianConstraints(c))
  {
    return boost::none;
  }

  // obtaining defaults
  int num_pos_constraints = c.position_constraints.size();
  int num_orient_constraints = c.orientation_constraints.size();
  int num_entries = num_pos_constraints >= num_orient_constraints ? num_pos_constraints : num_orient_constraints;
  std::string frame_id = !c.position_constraints.empty() ? c.position_constraints.front().header.frame_id : c.orientation_constraints.front().header.frame_id ;


  // creating default position constraint
  PositionConstraint default_pos_constraint;
  default_pos_constraint.header.frame_id = frame_id;
  default_pos_constraint.constraint_region.primitive_poses.resize(1);
  tf::poseEigenToMsg(Eigen::Affine3d::Identity(), default_pos_constraint.constraint_region.primitive_poses[0]);
  default_pos_constraint.constraint_region.primitive_poses[0].position.x = ref_pose.translation().x();
  default_pos_constraint.constraint_region.primitive_poses[0].position.y = ref_pose.translation().y();
  default_pos_constraint.constraint_region.primitive_poses[0].position.z = ref_pose.translation().z();
  shape_msgs::SolidPrimitive shape;
  shape.type = shape.SPHERE;
  shape.dimensions.push_back(default_pos_tol);
  default_pos_constraint.constraint_region.primitives.push_back(shape);

  // creating default orientation constraint
  OrientationConstraint default_orient_constraint;
  default_orient_constraint.header.frame_id = frame_id;
  default_orient_constraint.absolute_x_axis_tolerance = default_rot_tol;
  default_orient_constraint.absolute_y_axis_tolerance = default_rot_tol;
  default_orient_constraint.absolute_z_axis_tolerance = default_rot_tol;
  default_orient_constraint.orientation.x = default_orient_constraint.orientation.y = default_orient_constraint.orientation.z = 0;
  default_orient_constraint.orientation.w = 1;


  full_constraint.position_constraints.resize(num_entries);
  full_constraint.orientation_constraints.resize(num_entries);
  for(int i =0; i < num_entries; i++)
  {
    // populating position constraints
    if(c.position_constraints.size() >= num_entries)
    {
      full_constraint.position_constraints[i] = c.position_constraints[i];
    }
    else
    {
      full_constraint.position_constraints[i] = default_pos_constraint;
    }

    // populating orientation constraints
    if(c.orientation_constraints.size() >= num_entries)
    {
      full_constraint.orientation_constraints[i] = c.orientation_constraints[i];
    }
    else
    {
      full_constraint.orientation_constraints[i] = default_orient_constraint;
    }
  }

  return full_constraint;
}

bool decodeCartesianConstraint(moveit::core::RobotModelConstPtr model, const moveit_msgs::Constraints& constraints, Eigen::Affine3d& tool_pose,
                               Eigen::VectorXd& tolerance, std::string target_frame)
{
  std::vector<double> tolerance_std;
  if(!decodeCartesianConstraint(model,constraints,tool_pose,tolerance_std,target_frame))
  {
    return false;
  }

  tolerance = Eigen::VectorXd::Map(tolerance_std.data(),tolerance_std.size());
  return true;
}

bool decodeCartesianConstraint(moveit::core::RobotModelConstPtr model,const moveit_msgs::Constraints& constraints, Eigen::Affine3d& tool_pose,
                               std::vector<double>& tolerance, std::string target_frame)
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

  // transforming tool pose
  if(target_frame.empty())
  {
    target_frame = model->getModelFrame();
  }

  const moveit_msgs::PositionConstraint& pos_constraint = pos_constraints[0];
  std::string frame_id = pos_constraint.header.frame_id;
  moveit::core::RobotState state(model);
  state.update();
  if(!state.knowsFrameTransform(frame_id))
  {
    ROS_ERROR("Frame '%s' is not part of the model",frame_id.c_str());
    return false;
  }
  else if(!state.knowsFrameTransform(target_frame))
  {
    ROS_ERROR("Frame '%s' is not part of the model",target_frame.c_str());
    return false;
  }

  if(!frame_id.empty() && target_frame != frame_id)
  {
    Eigen::Affine3d root_to_frame = state.getFrameTransform(frame_id);
    Eigen::Affine3d root_to_target = state.getFrameTransform(target_frame);
    tool_pose = (root_to_target.inverse()) * root_to_frame * tool_pose;
  }

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

void computeTwist(const Eigen::Affine3d& p0,
                                        const Eigen::Affine3d& pf,
                                        const Eigen::ArrayXi& nullity,Eigen::VectorXd& twist)
{
  twist.resize(nullity.size());
  twist.setConstant(0);

  // relative transform
  auto p0_inv = p0.inverse();
  Eigen::Affine3d t = (p0_inv) * pf;

  Eigen::Vector3d twist_pos = pf.translation() - p0.translation();

  // relative rotation -> R = inverse(R0) * Rf
  Eigen::AngleAxisd relative_rot(t.rotation());
  double angle = relative_rot.angle();
  Eigen::Vector3d axis = relative_rot.axis();

  // forcing angle to range [-pi , pi]
  while( (angle > M_PI) || (angle < -M_PI))
  {
    angle = (angle >  M_PI) ? (angle - 2*M_PI) : angle;
    angle = (angle < -M_PI )? (angle + 2*M_PI) : angle;
  }

  // creating twist rotation relative to tool
  Eigen::Vector3d twist_rot = axis.normalized() * angle;

  // assigning into full 6dof twist vector
  twist.head(3) = twist_pos;
  twist.tail(3) = twist_rot;

  // zeroing all underconstrained cartesian dofs
  twist = (nullity == 0).select(0,twist);
}

void reduceJacobian(const Eigen::MatrixXd& jacb,
                                          const std::vector<int>& indices,Eigen::MatrixXd& jacb_reduced)
{
  jacb_reduced.resize(indices.size(),jacb.cols());
  for(auto i = 0u; i < indices.size(); i++)
  {
    jacb_reduced.row(i) = jacb.row(indices[i]);
  }
}


void calculateDampedPseudoInverse(const Eigen::MatrixXd &jacb, Eigen::MatrixXd &jacb_pseudo_inv,
                                         double eps, double lambda)
{
  using namespace Eigen;


  //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
  //in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<MatrixXd> svd(jacb, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const MatrixXd &U = svd.matrixU();
  const VectorXd &Sv = svd.singularValues();
  const MatrixXd &V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  // damp inverse with lambda so that inverse doesn't oscillate near solution
  size_t nSv = Sv.size();
  VectorXd inv_Sv(nSv);
  for(size_t i=0; i< nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
    {
      inv_Sv(i) = 1/Sv(i);
    }
    else
    {
      inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
    }
  }

  jacb_pseudo_inv = V * inv_Sv.asDiagonal() * U.transpose();
}

bool computeJacobianNullSpace(moveit::core::RobotStatePtr state,std::string group,std::string tool_link,
                                     const Eigen::ArrayXi& constrained_dofs,const Eigen::VectorXd& joint_pose,
                                     Eigen::MatrixXd& jacb_nullspace)
{
  using namespace Eigen;
  using namespace moveit::core;

  // robot state
  const JointModelGroup* joint_group = state->getJointModelGroup(group);
  state->setJointGroupPositions(joint_group,joint_pose);
  Affine3d tool_pose = state->getGlobalLinkTransform(tool_link);

  // jacobian calculations
  static MatrixXd jacb_transform(6,6);
  MatrixXd jacb, jacb_reduced, jacb_pseudo_inv;
  jacb_transform.setZero();

  if(!state->getJacobian(joint_group,state->getLinkModel(tool_link),Vector3d::Zero(),jacb))
  {
    ROS_ERROR("Failed to get Jacobian for link %s",tool_link.c_str());
    return false;
  }

  // transform jacobian rotational part to tool coordinates
  auto rot = tool_pose.inverse().rotation();
  jacb_transform.setZero();
  jacb_transform.block(0,0,3,3) = rot;
  jacb_transform.block(3,3,3,3) = rot;
  jacb = jacb_transform*jacb;

  // reduce jacobian and compute its pseudo inverse
  std::vector<int> indices;
  for(auto i = 0u; i < constrained_dofs.size(); i++)
  {
    if(constrained_dofs(i) != 0)
    {
      indices.push_back(i);
    }
  }
  reduceJacobian(jacb,indices,jacb_reduced);
  calculateDampedPseudoInverse(jacb_reduced,jacb_pseudo_inv,EPSILON,LAMBDA);

  int num_joints = joint_pose.size();
  jacb_nullspace = MatrixXd::Identity(num_joints,num_joints) - jacb_pseudo_inv*jacb_reduced;

  return true;
}

} // kinematics
} // utils
} // stomp_moveit




