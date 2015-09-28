cmake_minimum_required(VERSION 2.8.3)
project(constrained_ik)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS kdl_parser roscpp urdf eigen_conversions moveit_ros_planning moveit_core dynamic_reconfigure tf_conversions cmake_modules)

## System dependencies are found with CMake's conventions
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/ConstrainedIKDynamicReconfigure.cfg
  cfg/ConstrainedIKPlannerDynamicReconfigure.cfg
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
 LIBRARIES constrained_ik moveit_clik_planner_plugin constrained_ik_plugin
 CATKIN_DEPENDS roscpp urdf eigen_conversions tf_conversions urdf moveit_ros_planning moveit_core dynamic_reconfigure kdl_parser cmake_modules
 DEPENDS boost eigen orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

## Declare a cpp library
add_library(constrained_ik
            src/basic_kin.cpp
            src/constrained_ik.cpp
            src/constraint.cpp
            src/solver_state.cpp
            src/enum_types.cpp
            src/collision_robot_fcl_detailed.cpp
            src/constraints/avoid_joint_limits.cpp
            src/constraint_group.cpp
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
            src/constraints/goal_tool_pointing.cpp
)

target_link_libraries(constrained_ik ${catkin_LIBRARIES})

add_dependencies(constrained_ik ${PROJECT_NAME}_gencfg)

add_library(constrained_ik_plugin
            src/moveit_interface/constrained_ik_plugin.cpp
)
target_link_libraries(constrained_ik_plugin constrained_ik ${catkin_LIBRARIES})

add_library(moveit_clik_planner_plugin
            src/moveit_interface/constrained_ik_planner_plugin.cpp
            src/moveit_interface/joint_interpolation_planner.cpp
            src/moveit_interface/cartesian_planner.cpp)

target_link_libraries(moveit_clik_planner_plugin constrained_ik ${catkin_LIBRARIES})

## Declare a cpp executable
# add_executable(constrained_ik_node src/constrained_ik_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(constrained_ik_node constrained_ik_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS constrained_ik constrained_ik_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_basic_kin test/test_basic_kin.launch test/test_BasicKin.cpp )
  target_link_libraries(test_basic_kin constrained_ik)

  add_rostest_gtest(test_basic_ik test/test_basic_ik.launch test/test_BasicIK.cpp )
  target_link_libraries(test_basic_ik constrained_ik)

  add_rostest_gtest(test_constrained_ik
    test/test_constrained_ik.launch
    test/test_constrained_ik.cpp )
  target_link_libraries(test_constrained_ik constrained_ik)
endif()

