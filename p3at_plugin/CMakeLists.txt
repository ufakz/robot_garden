cmake_minimum_required(VERSION 2.8.3)
project(p3at_plugin)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf 
  std_msgs
  roscpp
  roslib
  move_base_msgs
  actionlib
  geometry_msgs
  nav_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

catkin_package(
  CATKIN_DEPENDS roscpp move_base_msgs actionlib geometry_msgs nav_msgs 
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIRS}
)

## Declare a cpp executable (our new controller)

add_executable(waypoint_following_movebase src/waypoint_following_movebase.cc include/get_flower_locations.h include/waypoints.h)
add_executable(waypoint_following_custom src/waypoint_following_custom.cc include/get_flower_locations.h include/waypoints.h)
add_executable(path_planning_movebase src/path_planning_movebase.cc include/get_flower_locations.h)

target_link_libraries(waypoint_following_movebase ${catkin_LIBRARIES} tinyxml2 ${roslib_LIBRARIES})
target_link_libraries(waypoint_following_custom ${catkin_LIBRARIES} tinyxml2 ${roslib_LIBRARIES})
target_link_libraries(path_planning_movebase ${catkin_LIBRARIES} tinyxml2 ${roslib_LIBRARIES})
