cmake_minimum_required(VERSION 2.8.3)
project(industrial_collision_detection)

add_definitions("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  pluginlib
  moveit_core
  pcl_ros
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(console_bridge REQUIRED)
find_package(octomap REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

# openvdb config
FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
find_library(OPENVDB_LIBRARY NAMES openvdb)
find_library(TBB_LIBRARY NAMES tbb)
find_library(HALF_LIBRARY NAMES Half)

# fcl config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
 LIBRARIES 
   ${PROJECT_NAME}
   ${PROJECT_NAME}_openvdb
   ${OPENVDB_LIBRARIES}
   ${OPENVDB_LIBRARY}
   ${TBB_LIBRARY}
   ${HALF_LIBRARY} 
 CATKIN_DEPENDS roscpp moveit_core cmake_modules pluginlib pcl_ros
 DEPENDS boost eigen fcl openvdb
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_openvdb 
  src/distance_field/openvdb_distance_field.cpp
  src/distance_field/openvdb_visualization.cpp)
target_link_libraries(${PROJECT_NAME}_openvdb ${catkin_LIBRARIES} ${OPENVDB_LIBRARY} ${TBB_LIBRARY} ${HALF_LIBRARY})

add_library(${PROJECT_NAME}
  src/collision_detection/collision_common.cpp
  src/collision_detection/collision_robot_industrial.cpp
  src/collision_detection/collision_world_industrial.cpp
  src/collision_detection/collision_robot_openvdb.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_openvdb 
  ${console_bridge_LIBRARIES} 
  ${urdfdom_LIBRARIES} 
  ${urdfdom_headers_LIBRARIES} 
  ${LIBFCL_LIBRARIES} 
  ${Boost_LIBRARIES})

add_library(${PROJECT_NAME}_plugin
  src/plugins/industrial_collision_detection.cpp
  src/plugins/openvdb_collision_detection.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
