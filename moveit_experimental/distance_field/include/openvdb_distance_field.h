#ifndef OPENVDB_DISTANCE_FIELD_H
#define OPENVDB_DISTANCE_FIELD_H

#include <geometric_shapes/shapes.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <moveit/robot_state/robot_state.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/math/Vec3.h>
#include <industrial_collision_detection/collision_common.h>

namespace distance_field
{

typedef std::vector<std::pair<openvdb::math::Vec3d, double> > SphereModel;
typedef boost::shared_ptr<std::vector<std::pair<openvdb::math::Vec3d, double> > > SphereModelPtr;

struct MeshData
{
  std::vector<openvdb::math::Vec3s> points;
  std::vector<openvdb::Vec3I> triangles;
  std::vector<openvdb::v2_1::Vec4I> quads;
};

class OpenVDBDistanceField
{

public:
    OpenVDBDistanceField(float voxel_size = 0.01, float background = 0.5);

    double getDistance(const Eigen::Vector3f &point, bool thread_safe = true) const;

    double getDistance(const openvdb::math::Coord &coord, bool thread_safe = true) const;

    double getDistance(const float &x, const float &y, const float &z, bool thread_safe = true) const;

    bool getGradient(const Eigen::Vector3f &point, Eigen::Vector3d &gradient, bool thread_safe = true) const;

    bool getGradient(const openvdb::math::Coord &coord, Eigen::Vector3d &gradient, bool thread_safe = true) const;

    bool getGradient(const float &x, const float &y, const float &z, Eigen::Vector3d &gradient, bool thread_safe = true) const;

    void addShapeToField(const shapes::Shape *shape,
                         const Eigen::Affine3d &pose,
                         const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                         const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);

    void addLinkToField(const robot_model::LinkModel *link,
                        const Eigen::Affine3d &pose,
                        const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                        const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);


    void fillWithSpheres(SphereModel &spheres,
                         int maxSphereCount,
                         bool overlapping = false,
                         float minRadius = 1.0,
                         float maxRadius = std::numeric_limits<float>::max(),
                         float isovalue = 0.0,
                         int instanceCount = 10000);

    void writeToFile(const std::string file_path);

    uint64_t memUsage() const;

    double getVoxelSize() const;

    openvdb::math::Transform::Ptr getTransform() const;

    openvdb::FloatGrid::Ptr getGrid() const;

private:

    float voxel_size_;
    float background_;
    openvdb::FloatGrid::Ptr grid_;
    std::shared_ptr<openvdb::FloatGrid::ConstAccessor> accessor_;
    openvdb::math::Transform::Ptr transform_;
};

typedef boost::shared_ptr<OpenVDBDistanceField> OpenVDBDistanceFieldPtr;
typedef boost::shared_ptr<const OpenVDBDistanceField> OpenVDBDistanceFieldConstPtr;


MeshData ShapeMeshToOpenVDB(const shapes::Mesh *mesh, const Eigen::Affine3d &pose);

void Affine3dToMat4d(const Eigen::Affine3d &input, openvdb::math::Mat4d &output);

void Affine3dToMat4dAffine(const Eigen::Affine3d &input, openvdb::math::Mat4d &output);

void WorldToIndex(const openvdb::math::Transform::Ptr transform, std::vector<openvdb::math::Vec3s> &points);

void TransformVec3s(const Eigen::Affine3d &pose, std::vector<openvdb::math::Vec3s> &points);

struct DistanceQueryData
{
  OpenVDBDistanceFieldConstPtr sdf;
  SphereModelPtr spheres;
  std::string link_name[2];
  openvdb::math::Transform::Ptr transform;
};

class CollisionRobotOpenVDB
{
public:
  CollisionRobotOpenVDB(const robot_model::RobotModelConstPtr &model,
                        const float voxel_size = 0.01,
                        const float background = 0.5,
                        const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                        const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);

//  double distanceSelf(const robot_state::RobotState &state) const;
  void distanceSelf(const collision_detection::DistanceRequest &req, collision_detection::DistanceResult &res, const robot_state::RobotState &state) const;

  void writeToFile(const std::string file_path);

  uint64_t memUsage() const;

private:
  void distanceSelfHelper(DistanceQueryData &data, double &min_dist, openvdb::math::Vec3d &location) const;

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
   * @brief This is a recursive helper function that creates the static signed distance field given a base link.
   *
   * @param link, The link to search for child links that are attached by fixed transform.
   */
  void addAssociatedFixedTransforms(const robot_model::LinkModel *link);

  bool isCollisionAllowed(const std::string &l1, const std::string &l2, const collision_detection::AllowedCollisionMatrix *acm) const;

  const robot_model::RobotModelConstPtr robot_model_;

  const std::vector<const robot_model::LinkModel*>& links_;

  std::vector<OpenVDBDistanceFieldConstPtr> static_sdf_;
  std::vector<const robot_model::LinkModel*> static_links_;

  std::vector<OpenVDBDistanceFieldConstPtr> dynamic_sdf_;
  std::vector<const robot_model::LinkModel*> dynamic_links_;

  std::vector<OpenVDBDistanceFieldConstPtr> active_sdf_;
  std::vector<const robot_model::LinkModel*> active_links_;
  std::vector<SphereModelPtr> active_spheres_;

  float voxel_size_;
  float background_;
  float exBandWidth_;
  float inBandWidth_;
};

}

#endif //OPENVDB_DISTANCE_FIELD_H
