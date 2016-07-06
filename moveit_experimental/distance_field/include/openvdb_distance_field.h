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

    double getDistance(Eigen::Vector3f &point);

    void addShapeToField(const shapes::Shape* shape,
                         const Eigen::Affine3d &pose,
                         const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                         const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);

    void addRobotToField(const moveit::core::RobotStatePtr robot_state,
                         const float exBandWidth = openvdb::LEVEL_SET_HALF_WIDTH,
                         const float inBandWidth = openvdb::LEVEL_SET_HALF_WIDTH);

    void writeToFile(const std::string file_path);

    openvdb::FloatGrid::Ptr getGrid() const;

private:
    float voxel_size_;
    float background_;
    openvdb::FloatGrid::Ptr grid_;
    openvdb::math::Transform::Ptr transform_;

};

static MeshData ShapeMeshToOpenVDB(const shapes::Mesh *mesh, const Eigen::Affine3d &pose);
static void Affine3dToMat4d(const Eigen::Affine3d &input, openvdb::math::Mat4d &output);
static void WorldToIndex(const openvdb::math::Transform::Ptr transform, std::vector<openvdb::math::Vec3s> &points);
static void TransformVec3s(const Eigen::Affine3d &pose, std::vector<openvdb::math::Vec3s> &points);

/**
 * @brief This returns the minimum value between two grid values
 * @param a First grid value
 * @param b Second grid value
 * @param result The minimum value between a and b
 */
static inline void diff(const float& a, const float& b, float& result) {result = std::min(a, b);}

}

#endif //OPENVDB_DISTANCE_FIELD_H
