#ifndef OPENVDB_DISTANCE_FIELD_H
#define OPENVDB_DISTANCE_FIELD_H

#include <geometric_shapes/shapes.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <moveit/robot_state/robot_state.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/math/Vec3.h>

namespace distance_field
{

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

    double getDistance(const Eigen::Vector3f &point) const;

    double getDistance(const openvdb::math::Coord &coord) const;

    double getDistance(const float &x, const float &y, const float &z) const;

    double getDistanceGradient(const Eigen::Vector3f &point, Eigen::Vector3f &gradient) const;

    double getDistanceGradient(const openvdb::math::Coord &coord, Eigen::Vector3f &gradient) const;

    double getDistanceGradient(const float &x, const float &y, const float &z, Eigen::Vector3f &gradient) const;

    void addShapeToField(const shapes::Shape* shape,
                         const Eigen::Affine3d &pose,
                         const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                         const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);



    void writeToFile(const std::string file_path);

    openvdb::FloatGrid::Ptr getGrid() const;

private:

    float voxel_size_;
    float background_;
    openvdb::FloatGrid::Ptr grid_;
    std::shared_ptr<openvdb::FloatGrid::ConstAccessor> accessor_;
    openvdb::math::Transform::Ptr transform_;
};

static MeshData ShapeMeshToOpenVDB(const shapes::Mesh *mesh, const Eigen::Affine3d &pose);

static void Affine3dToMat4d(const Eigen::Affine3d &input, openvdb::math::Mat4d &output);

static void WorldToIndex(const openvdb::math::Transform::Ptr transform, std::vector<openvdb::math::Vec3s> &points);

static void TransformVec3s(const Eigen::Affine3d &pose, std::vector<openvdb::math::Vec3s> &points);


class CollisionRobotOpenVDB
{
public:
  CollisionRobotOpenVDB(const robot_model::RobotModelConstPtr &model,
                        const float voxel_size = 0.01,
                        const float background = 0.5,
                        const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                        const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);

  void writeToFile(const std::string file_path);

private:
  void addRobotToField();

  void addLinkToField(const moveit::core::LinkModel *link, const Eigen::Affine3d &pose, OpenVDBDistanceField &sdf);

  const robot_model::RobotModelConstPtr robot_model_;

  OpenVDBDistanceField static_objects; /**< A single SDF that contains all fix objects */

  std::vector<OpenVDBDistanceField> dynamic_objects; /**< A vector of SDF that contains all dynamic objects */

  float exBandWidth_;

  float inBandWidth_;
};

}

#endif //OPENVDB_DISTANCE_FIELD_H
