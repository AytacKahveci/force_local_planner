cmake_minimum_required(VERSION 3.0.2)
project(force_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  costmap_converter
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  angles
  tf
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
 LIBRARIES force_local_planner
 CATKIN_DEPENDS 
  base_local_planner 
  costmap_2d 
  costmap_converter 
  geometry_msgs 
  nav_core 
  nav_msgs 
  pluginlib 
  roscpp 
  tf2 
  tf2_ros
 #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EXTERNAL_INCLUDE_DIRS}
)

add_library(force_local_planner 
  src/force_local_planner.cpp
  src/laser_model.cpp
  src/traj_generator.cpp)

target_link_libraries(force_local_planner
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS force_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  force_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)