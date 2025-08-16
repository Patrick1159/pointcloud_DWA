cmake_minimum_required(VERSION 3.0.2)
project(dwa_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  tf2
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dwa_demo
 CATKIN_DEPENDS roscpp pcl_conversions pcl_ros tf2 tf2_geometry_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(dwa_planner 
  src/main.cpp
  include/dwa_demo/DWA.cpp
  # include/utils/visualization.h
)

target_link_libraries(dwa_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS dwa_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

