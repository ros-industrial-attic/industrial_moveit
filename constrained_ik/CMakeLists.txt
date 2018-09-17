cmake_minimum_required(VERSION 2.8.12)
project(constrained_ik)

if(CMAKE_VERSION VERSION_LESS 3.1)
    add_compile_options(-std=c++11)
else()
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS ON)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
              kdl_parser
              roscpp
              urdf
              eigen_conversions
              moveit_ros_planning
              moveit_core
              dynamic_reconfigure
              tf_conversions
              cmake_modules)

## System dependencies are found with CMake's conventions
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)

# We don't build anything here, but in order to export the right
# build flags in catkin_package(), we still have to find Eigen3 here.  Note
# that this is somewhat complicated because of our need to support Ubuntu
# all the way back to saucy.  First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu.  If that fails, then we
# fall-back to the version provided by cmake_modules, which is a stand-in.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # saucy does not have Eigen3, but have Eigen instead
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/CLIKDynamic.cfg
  cfg/CLIKPlannerDynamic.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
 LIBRARIES constrained_ik constrained_ik_constraints constrained_ik_moveit
 CATKIN_DEPENDS roscpp urdf eigen_conversions tf_conversions urdf moveit_ros_planning moveit_core dynamic_reconfigure kdl_parser cmake_modules
 DEPENDS Boost EIGEN3 orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a cpp library
add_library(constrained_ik
            src/basic_kin.cpp
            src/constrained_ik.cpp
            src/constraint.cpp
            src/solver_state.cpp
            src/enum_types.cpp
            src/constrained_ik_utils.cpp
            src/constraint_group.cpp
)

target_link_libraries(constrained_ik ${catkin_LIBRARIES})

add_library(constrained_ik_constraints
            src/constraints/goal_orientation.cpp
            src/constraints/goal_position.cpp
            src/constraints/tool_position.cpp
            src/constraints/goal_tool_orientation.cpp
            src/constraints/goal_minimize_change.cpp
            src/constraints/goal_mid_joint.cpp
            src/constraints/goal_zero_jvel.cpp
            src/constraints/avoid_singularities.cpp
            src/constraints/joint_vel_limits.cpp
            src/constraints/avoid_obstacles.cpp
            src/constraints/avoid_joint_limits.cpp
            src/constraints/goal_tool_pointing.cpp
)
target_link_libraries(constrained_ik_constraints constrained_ik ${catkin_LIBRARIES})

add_dependencies(constrained_ik ${PROJECT_NAME}_gencfg)

add_library(constrained_ik_plugin
            src/moveit_interface/constrained_ik_plugin.cpp
)
target_link_libraries(constrained_ik_plugin constrained_ik constrained_ik_constraints ${catkin_LIBRARIES})

add_library(constrained_ik_moveit
            src/moveit_interface/joint_interpolation_planner.cpp
            src/moveit_interface/cartesian_planner.cpp)

target_link_libraries(constrained_ik_moveit constrained_ik constrained_ik_constraints ${catkin_LIBRARIES})

add_library(moveit_clik_planner_plugin
            src/moveit_interface/constrained_ik_planner_plugin.cpp)

target_link_libraries(moveit_clik_planner_plugin constrained_ik constrained_ik_constraints constrained_ik_moveit ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS constrained_ik LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS constrained_ik_constraints LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS constrained_ik_moveit LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(
  FILES
    constrained_ik_constraint_plugin_description.xml
    constrained_ik_planner_plugin_description.xml
    constrained_ik_plugin.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS constrained_ik_plugin moveit_clik_planner_plugin
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_basic_kin test/test_basic_kin.launch test/test_basic_kin.cpp )
  target_link_libraries(test_basic_kin constrained_ik constrained_ik_constraints)

  add_rostest_gtest(test_constrained_ik
    test/test_constrained_ik.launch
    test/test_constrained_ik.cpp )
  target_link_libraries(test_constrained_ik constrained_ik constrained_ik_constraints)
endif()

##############
## Examples ##
##############

## Add cpp example targets and link libraries
SET(EXAMPLE_LIBS
    constrained_ik
    constrained_ik_constraints
    ${catkin_LIBRARIES})

add_executable(constraints_from_yaml_example examples/constraints_from_yaml_example.cpp examples/example_utils.h)
target_link_libraries(constraints_from_yaml_example ${EXAMPLE_LIBS})

add_executable(avoid_joint_limits_example examples/avoid_joint_limits_example.cpp examples/example_utils.h)
target_link_libraries(avoid_joint_limits_example ${EXAMPLE_LIBS})

add_executable(avoid_obstacles_example examples/avoid_obstacles_example.cpp examples/example_utils.h)
target_link_libraries(avoid_obstacles_example ${EXAMPLE_LIBS})

add_executable(avoid_singularities_example examples/avoid_singularities_example.cpp examples/example_utils.h)
target_link_libraries(avoid_singularities_example ${EXAMPLE_LIBS})

add_executable(goal_mid_joint_example examples/goal_mid_joint_example.cpp examples/example_utils.h)
target_link_libraries(goal_mid_joint_example ${EXAMPLE_LIBS})

add_executable(goal_minimize_change_example examples/goal_minimize_change_example.cpp examples/example_utils.h)
target_link_libraries(goal_minimize_change_example ${EXAMPLE_LIBS})

add_executable(goal_orientation_example examples/goal_orientation_example.cpp examples/example_utils.h)
target_link_libraries(goal_orientation_example ${EXAMPLE_LIBS})

add_executable(goal_position_example examples/goal_position_example.cpp examples/example_utils.h)
target_link_libraries(goal_position_example ${EXAMPLE_LIBS})

add_executable(goal_tool_orientation_example examples/goal_tool_orientation_example.cpp examples/example_utils.h)
target_link_libraries(goal_tool_orientation_example ${EXAMPLE_LIBS})

add_executable(goal_tool_pointing_example examples/goal_tool_pointing_example.cpp examples/example_utils.h)
target_link_libraries(goal_tool_pointing_example ${EXAMPLE_LIBS})

add_executable(goal_zero_jvel_example examples/goal_zero_jvel_example.cpp examples/example_utils.h)
target_link_libraries(goal_zero_jvel_example ${EXAMPLE_LIBS})

add_executable(avoid_joint_vel_limits_example examples/avoid_joint_vel_limits_example.cpp examples/example_utils.h)
target_link_libraries(avoid_joint_vel_limits_example ${EXAMPLE_LIBS})

add_executable(goal_tool_position_example examples/goal_tool_position_example.cpp examples/example_utils.h)
target_link_libraries(goal_tool_position_example ${EXAMPLE_LIBS})
