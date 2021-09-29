cmake_minimum_required(VERSION 3.0.2)
project(industrial_moveit_test_support)

## Find catkin macros and libraries
find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############
install(DIRECTORY meshes urdf
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

