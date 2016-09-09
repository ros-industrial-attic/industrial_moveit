#ifndef COLLISION_ROBOT_OPENVDB_H
#define COLLISION_ROBOT_OPENVDB_H

#include <industrial_collision_detection/collision_detection/collision_common.h>
#include <industrial_collision_detection/distance_field/openvdb_distance_field.h>
#include <industrial_collision_detection/collision_detection/collision_robot_industrial.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <memory>

namespace collision_detection
{

enum SDFType {Static = 0, Dynamic = 1, Active = 2};
struct DistanceQueryData
{
  DistanceQueryData() :
    empty(true),
    gradient(false) {}

  std::string parent_name;
  distance_field::SphereModel spheres;
  std::vector<std::string> child_name;
  std::vector<int> child_index;
  std::vector<int> child_type;
  bool gradient;
  bool empty;
};

struct SDFData
{
  SDFData(openvdb::FloatGrid::Ptr sdf, openvdb::Mat4d &tf) : accessor(sdf->getConstAccessor())
  {
    transform = openvdb::math::Transform::createLinearTransform(tf);
    transform->preScale(sdf->transformPtr()->voxelSize());
  }

  SDFData(openvdb::FloatGrid::Ptr sdf) :
    accessor(sdf->getConstAccessor()),
    transform(sdf->transformPtr()) {}

  openvdb::math::Transform::Ptr transform;
  openvdb::FloatGrid::ConstAccessor accessor;
};

class CollisionRobotOpenVDB: public CollisionRobotIndustrial
{
public:
  /**
   * @brief CollisionRobotOpenVDB constructor which uses a distance field and spherical approximations to compute distances.
   * Collision checks are made with the FCL library.
   *
   */
  CollisionRobotOpenVDB(const robot_model::RobotModelConstPtr &model,
                        const float voxel_size = 0.01,
                        const float background = 0.5,
                        const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                        const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);

  /**
   * @brief Alternative constructor that loads an already generated set of distance
   * fields given a particular robot model.
   * @param model The robot model with which to load and interpret saved fields
   * @param file_path The .vdb file that containts the archived distance fields
   */
  CollisionRobotOpenVDB(const robot_model::RobotModelConstPtr& model,
                        const std::string& file_path);

  /**
   * @brief overloaded CollisionRobot::checkSelfCollision methods
   */
  virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const override;
  virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state
                                  , const AllowedCollisionMatrix &acm) const override;

  /**
   * @brief customized distanceSelf method.  Returns the shortest distance between a robot link in the
   * requested planning group and any given obstacle in the environment.  This calculation relies on
   * the distance field and the spherical approximation for each link.
   */
  void distanceSelf(const collision_detection::DistanceRequest &req,
                    collision_detection::DistanceResult &res, const robot_state::RobotState &state) const;

  uint64_t memUsage() const;

  void writeToFile(const std::string &file_path);

  /**
   * @brief Returns a pair of 'inside' & 'outside' distance clouds.
   * @param state The robot state to visualize
   * @return Pair of inside & outside surface point clouds visualizing voxels
   */
  std::pair<distance_field::PointCloud::Ptr, distance_field::PointCloud::Ptr>
  voxelGridToPointClouds(const moveit::core::RobotState& state) const;

  std::pair<distance_field::PointCloud::Ptr, distance_field::PointCloud::Ptr>
  voxelGridToPointClouds(const moveit::core::RobotState& state,
                         const std::vector<std::string>& exclude_list) const;

  visualization_msgs::MarkerArray spheresToVisualizationMarkers(const moveit::core::RobotState& state) const;

private:

  std::pair<openvdb::GridPtrVecPtr, openvdb::MetaMap::Ptr> readFromFile(const std::string& file_path);

  void distanceSelfHelper(const DistanceQueryData &data, std::vector<std::vector<SDFData> > &sdfs_data, collision_detection::DistanceResultsData &res) const;


  bool hasCollisionGeometry(const robot_model::LinkModel* link) const;

  std::vector<const robot_model::LinkModel*> identifyStaticLinks() const;
  void identifyStaticLinksHelper(const robot_model::LinkModel* link,
                                 std::vector<const robot_model::LinkModel*>& in_set,
                                 std::vector<const robot_model::LinkModel*>& considered) const;


  std::vector<const robot_model::LinkModel*> identifyActiveLinks() const;

  std::vector<const robot_model::LinkModel*> identifyDynamicLinks(std::vector<const moveit::core::LinkModel *> &static_links, std::vector<const moveit::core::LinkModel *> &active_links) const;

  void loadStaticLinks(std::vector<const robot_model::LinkModel*>& static_links,
                       const openvdb::GridPtrVec& grids,
                       std::vector<distance_field::OpenVDBDistanceFieldConstPtr> &fields);

  void loadActiveLinks(std::vector<const robot_model::LinkModel*>& active_links,
                       const openvdb::GridPtrVec& grids, std::vector<distance_field::OpenVDBDistanceFieldConstPtr> &fields);

  void loadDynamicLinks(std::vector<const robot_model::LinkModel*>& dynamic_links,
                        const openvdb::GridPtrVec& grids, std::vector<distance_field::OpenVDBDistanceFieldConstPtr> &fields);

  /**
   * @brief Create static signed distance fields.
   *
   * These are links attached to the world by fixed transforms or by proxy
   */
  void createStaticSDFs();

  /**
   * @brief Create active signed distance fields.
   *
   * These are links that are associated to a move_group (joint, linear, etc.). These are
   * the only links that get a sphere model.
   */
  void createActiveSDFs();

  /**
   * @brief Create dynamic signed distance fields.
   *
   * These are links that have the ability to move but are not associated to a move_group (joint, linear, etc.)
   */
  void createDynamicSDFs();

  /**
   * @brief Generate a default distance query structure.
   */
  void createDefaultDistanceQuery();

  /**
   * @brief generateDefaultAllowedCollisionMatrix
   */
  void createDefaultAllowedCollisionMatrix();

  bool isCollisionAllowed(const std::string &l1, const std::string &l2, const collision_detection::AllowedCollisionMatrix *acm) const;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  const std::vector<const robot_model::LinkModel*>& links_;

  std::vector<distance_field::OpenVDBDistanceFieldConstPtr> static_sdf_;
  std::vector<const robot_model::LinkModel*> static_links_;

  std::vector<distance_field::OpenVDBDistanceFieldConstPtr> dynamic_sdf_;
  std::vector<const robot_model::LinkModel*> dynamic_links_;

  std::vector<distance_field::OpenVDBDistanceFieldConstPtr> active_sdf_;
  std::vector<const robot_model::LinkModel*> active_links_;
  std::vector<distance_field::SphereModel> active_spheres_;

  std::vector<DistanceQueryData> dist_query_;

  float voxel_size_;
  float background_;
  float exBandWidth_;
  float inBandWidth_;
};

}

#endif // COLLISION_ROBOT_OPENVDB_H
