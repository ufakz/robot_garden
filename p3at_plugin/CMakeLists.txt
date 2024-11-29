cmake_minimum_required(VERSION 2.8.3)
project(p3at_plugin)

# find_package(catkin REQUIRED COMPONENTS geometry_msgs tf std_msgs move_base_msgs actionlib)

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

# catkin_package( CATKIN_DEPENDS roscpp)

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
add_executable(p3at src/p3at_controller_movebase.cc)
add_executable(p3at_path_planning src/p3at_path_planning.cc src/get_flower_locations.cc)
target_link_libraries(p3at ${catkin_LIBRARIES} tinyxml2)
target_link_libraries(p3at_path_planning ${catkin_LIBRARIES} tinyxml2 ${roslib_LIBRARIES})

## add_executable(actionlib src/actionlib_movebase.cc)
## target_link_libraries(actionlib ${catkin_LIBRARIES})

##add_executable(p3at src/p3at.cc)
##target_link_libraries(p3at ${catkin_LIBRARIES})

install(TARGETS p3at
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
