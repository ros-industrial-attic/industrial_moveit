/**
 * @file static_distance_field.h
 * @brief This is a static distance field
 *
 * This allows the user to import data from a file. The data is only
 * able to be loaded from a file, user is unable to change the data via
 * methods: addPointsToField, removePointsFromField, updatePointsInField.
 *
 * @author Levi Armstrong
 * @date June 22, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
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
#ifndef STATIC_DISTANCE_FIELD_H
#define STATIC_DISTANCE_FIELD_H

#include <moveit/distance_field/distance_field.h>
#include <moveit/distance_field/voxel_grid.h>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <console_bridge/console.h>
#include <type_traits>
#include <ros/ros.h>

namespace distance_field
{

/**
 * @brief This class represents a static distance field which can not be modified.
 *
 * This is used in cases where the environment is static to allow for user the ability
 * to have a higher fidelity distance field with less memory usage than the propagation
 * distance field.
 */
template <typename Floating>
class StaticDistanceField : public DistanceField
{
    static_assert(std::is_floating_point<Floating>::value,"Static Distance Field only allows floating point types.");

public:
  StaticDistanceField();
  StaticDistanceField(std::istream& stream);
  StaticDistanceField(const DistanceField &df);

  virtual double getDistance(double x, double y, double z) const override;
  virtual double getDistance(int x, int y, int z) const override;
  virtual bool isCellValid(int x, int y, int z) const override;
  virtual int getXNumCells() const override;
  virtual int getYNumCells() const override;
  virtual int getZNumCells() const override;

  virtual bool gridToWorld(int x, int y, int z,
                              double& world_x, double& world_y, double& world_z) const override;

  virtual bool worldToGrid(double world_x, double world_y, double world_z,
                              int& x, int& y, int& z) const override;

  virtual bool writeToStream(std::ostream& stream) const override;

  virtual bool readFromStream(std::istream& stream) override;

  /**
   * @brief This will populate the static distance field using an existing distance field.
   * @param df DistanceField
   * @return bool
   */
  bool readFromDistanceField(const DistanceField &df);

  virtual double getUninitializedDistance() const override;

  virtual void reset() override {}

private:
  void addPointsToField(const EigenSTL::vector_Vector3d &points) override;
  void removePointsFromField(const EigenSTL::vector_Vector3d &points) override;
  void updatePointsInField(const EigenSTL::vector_Vector3d& old_points,
                                      const EigenSTL::vector_Vector3d& new_points) override;

  template<typename S>
  bool readFromStreamHelper(std::istream& stream);

  double max_distance_;
  boost::shared_ptr<distance_field::VoxelGrid<Floating> > voxel_grid_;
};

/**
 * @brief This will export any distance field as static.
 * @param stream file to write to
 * @param df distance field to export as static.
 * @return bool
 */
template<typename Floating>
static bool writeToStreamStatic(std::ostream &stream, const DistanceField &df);
}

template<typename Floating>
distance_field::StaticDistanceField<Floating>::StaticDistanceField()
  : DistanceField(0, 0, 0, 0, 0, 0, 0)
{
  static_assert(std::is_floating_point<Floating>::value,"Error static distance field");
}

template<typename Floating>
distance_field::StaticDistanceField<Floating>::StaticDistanceField(std::istream &stream)
  : DistanceField(0, 0, 0, 0, 0, 0, 0)
{
  static_assert(std::is_floating_point<Floating>::value,"Error static distance field");
  readFromStream(stream);
}

template<typename Floating>
distance_field::StaticDistanceField<Floating>::StaticDistanceField(const DistanceField &df)
  : DistanceField(0, 0, 0, 0, 0, 0, 0)
{
  static_assert(std::is_floating_point<Floating>::value,"Error static distance field");
  readFromDistanceField(df);
}

template<typename Floating>
void distance_field::StaticDistanceField<Floating>::addPointsToField(const EigenSTL::vector_Vector3d &points)
{
  logWarn("Method addPointsToField is not implemented for static distance field.");
}

template<typename Floating>
void distance_field::StaticDistanceField<Floating>::removePointsFromField(const EigenSTL::vector_Vector3d &points)
{
  logWarn("Method removePointsFromField is not implemented for static distance field.");
}

template<typename Floating>
void distance_field::StaticDistanceField<Floating>::updatePointsInField(const EigenSTL::vector_Vector3d& old_points,
                                    const EigenSTL::vector_Vector3d& new_points)
{
  logWarn("Method updatePointsInField is not implemented for static distance field.");
}

template<typename Floating>
double distance_field::StaticDistanceField<Floating>::getDistance(double x, double y, double z) const
{
  return (*voxel_grid_.get())(x,y,z);
}

template<typename Floating>
double distance_field::StaticDistanceField<Floating>::getDistance(int x, int y, int z) const
{
  return voxel_grid_->getCell(x,y,z);
}

template<typename Floating>
bool distance_field::StaticDistanceField<Floating>::isCellValid(int x, int y, int z) const
{
  return voxel_grid_->isCellValid(x,y,z);
}

template<typename Floating>
int distance_field::StaticDistanceField<Floating>::getXNumCells() const
{
  return voxel_grid_->getNumCells(distance_field::DIM_X);
}

template<typename Floating>
int distance_field::StaticDistanceField<Floating>::getYNumCells() const
{
  return voxel_grid_->getNumCells(distance_field::DIM_Y);
}

template<typename Floating>
int distance_field::StaticDistanceField<Floating>::getZNumCells() const
{
  return voxel_grid_->getNumCells(distance_field::DIM_Z);
}

template<typename Floating>
bool distance_field::StaticDistanceField<Floating>::gridToWorld(int x, int y, int z,
                            double& world_x, double& world_y, double& world_z) const
{
  voxel_grid_->gridToWorld(x, y, z, world_x, world_y, world_z);
  return true;
}

template<typename Floating>
bool distance_field::StaticDistanceField<Floating>::worldToGrid(double world_x, double world_y, double world_z,
                            int& x, int& y, int& z) const
{
  voxel_grid_->worldToGrid(world_x, world_y, world_z, x, y, z);
  return true;
}

template<typename Floating>
double distance_field::StaticDistanceField<Floating>::getUninitializedDistance() const
{
  return max_distance_;
}

template<typename Floating>
bool distance_field::StaticDistanceField<Floating>::writeToStream(std::ostream &stream) const
{
  writeToStreamStatic<Floating>(stream, *this);
}

template<typename Floating>
bool distance_field::StaticDistanceField<Floating>::readFromStream(std::istream &stream)
{
  std::string float_type_name;
  double version;

  if(!stream.good()) return false;

  std::string temp;

  stream >> temp;
  if(temp != "version:") return false;
  stream >> version;

  stream >> temp;
  if(temp != "resolution:") return false;
  stream >> resolution_;

  stream >> temp;
  if(temp != "size_x:") return false;
  stream >> size_x_;

  stream >> temp;
  if(temp != "size_y:") return false;
  stream >> size_y_;

  stream >> temp;
  if(temp != "size_z:") return false;
  stream >> size_z_;

  stream >> temp;
  if(temp != "origin_x:") return false;
  stream >> origin_x_;

  stream >> temp;
  if(temp != "origin_y:") return false;
  stream >> origin_y_;

  stream >> temp;
  if(temp != "origin_z:") return false;
  stream >> origin_z_;

  stream >> temp;
  if(temp != "max_distance:") return false;
  stream >> max_distance_;

  stream >> temp;
  if(temp != "float_type:") return false;
  stream >> float_type_name;

  //this should be newline
  char nl;
  stream.get(nl);

  inv_twice_resolution_ = 1.0/(2.0*resolution_);

  voxel_grid_.reset(new distance_field::VoxelGrid<Floating>(size_x_, size_y_, size_z_, resolution_, origin_x_, origin_y_, origin_z_, max_distance_));

  if (float_type_name == std::string(typeid(float).name()))
    return readFromStreamHelper<float>(stream);
  else if(float_type_name == std::string(typeid(double).name()))
    return readFromStreamHelper<double>(stream);
  else if(float_type_name == std::string(typeid(long double).name()))
     return readFromStreamHelper<long double>(stream);
  else
    return false;

}

template<typename Floating>
template<typename S>
bool distance_field::StaticDistanceField<Floating>::readFromStreamHelper(std::istream &stream)
{
  //now we start the compressed portion
  boost::iostreams::filtering_istream in;
  in.push(boost::iostreams::zlib_decompressor());
  in.push(stream);
  std::size_t size = sizeof(S);

  for(unsigned int x = 0; x < static_cast<unsigned int>(getXNumCells()); x++)
  {
    for(unsigned int y = 0; y < static_cast<unsigned int>(getYNumCells()); y++)
    {
      for(unsigned int z = 0; z < static_cast<unsigned int>(getZNumCells()); z++)
      {
        S dist;
        in.read(reinterpret_cast<char *>(&dist), size);
        voxel_grid_->setCell(x,y,z, static_cast<Floating>(dist));
      }
    }
  }

  return true;
}

template<typename Floating>
bool distance_field::StaticDistanceField<Floating>::readFromDistanceField(const DistanceField &df)
{
  resolution_ = df.getResolution();
  size_x_ = df.getSizeX();
  size_y_ = df.getSizeY();;
  size_z_ = df.getSizeZ();;
  origin_x_ = df.getOriginX();
  origin_y_ = df.getOriginY();;
  origin_z_ = df.getOriginZ();;
  max_distance_ = df.getUninitializedDistance();
  inv_twice_resolution_ = 1.0/(2.0*resolution_);
  voxel_grid_.reset(new distance_field::VoxelGrid<Floating>(size_x_, size_y_, size_z_, resolution_, origin_x_, origin_y_, origin_z_, max_distance_));

  for(int x = 0; x < static_cast<unsigned int>(getXNumCells()); x++)
  {
    for(int y = 0; y < static_cast<unsigned int>(getYNumCells()); y++)
    {
      for(int z = 0; z < static_cast<unsigned int>(getZNumCells()); z++)
      {
        Floating dist = static_cast<Floating>(df.getDistance(x,y,z));
        voxel_grid_->setCell(x,y,z, dist);
      }
    }
  }

  return true;
}

template<typename Floating>
bool distance_field::writeToStreamStatic(std::ostream &stream, const DistanceField &df)
{
  static_assert(std::is_floating_point<Floating>::value, "StaticDistanceField::writeToStreamStatic only allows floating point types.");

  std::string type_name = typeid(Floating).name();
  double version = 1.0;

  stream << "version: " << version << std::endl;
  stream << "resolution: " << df.getResolution() << std::endl;
  stream << "size_x: " << df.getSizeX() << std::endl;
  stream << "size_y: " << df.getSizeY() << std::endl;
  stream << "size_z: " << df.getSizeZ() << std::endl;
  stream << "origin_x: " << df.getOriginX() << std::endl;
  stream << "origin_y: " << df.getOriginY() << std::endl;
  stream << "origin_z: " << df.getOriginZ() << std::endl;
  stream << "max_distance: " << df.getUninitializedDistance() << std::endl;
  stream << "float_type: " << type_name << std::endl;
  //now the binary stuff

  //first writing to zlib compressed buffer
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::zlib_compressor());
  out.push(stream);
  std::size_t size = sizeof(Floating);

  for(int x = 0; x < df.getXNumCells(); x++)
  {
    for(int y = 0; y < df.getYNumCells(); y++)
    {
      for(int z = 0; z < df.getZNumCells(); z++)
      {
        Floating dist = static_cast<Floating>(df.getDistance(x,y,z));
        out.write(reinterpret_cast<char *>(&dist), size);
      }
    }
  }
  out.flush();
  return true;
}

#endif // STATIC_DISTANCE_FIELD_H
