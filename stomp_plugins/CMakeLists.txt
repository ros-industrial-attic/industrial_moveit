cmake_minimum_required(VERSION 2.8.3)
project(stomp_plugins)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  stomp_core
  stomp_moveit
  cmake_modules
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_definitions("-std=c++11")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_planning moveit_core stomp_core stomp_moveit cmake_modules pluginlib roscpp
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# cost function plugin(s)
add_library(${PROJECT_NAME}_cost_functions
  src/cost_functions/tool_goal_pose.cpp
 )
target_link_libraries(${PROJECT_NAME}_cost_functions ${catkin_LIBRARIES})

# noise generator plugin(s)
add_library(${PROJECT_NAME}_noise_generators
  src/noise_generators/goal_guided_multivariate_gaussian.cpp
 )
target_link_libraries(${PROJECT_NAME}_noise_generators ${catkin_LIBRARIES})

# update plugin(s)
add_library(${PROJECT_NAME}_update_filters
  src/update_filters/constrained_cartesian_goal.cpp
 )
target_link_libraries(${PROJECT_NAME}_update_filters ${catkin_LIBRARIES})

#############
## Install ##
#############
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_cost_functions ${PROJECT_NAME}_noise_generators ${PROJECT_NAME}_update_filters
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES cost_function_plugins.xml noise_generator_plugins.xml update_filter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
