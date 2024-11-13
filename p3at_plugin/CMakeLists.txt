cmake_minimum_required(VERSION 2.8.3)
project(p3at_plugin)

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf std_msgs)

find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

catkin_package(
  CATKIN_DEPENDS roscpp)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIRS}
  )

## Declare a cpp executable (our new controller)
add_executable(p3at src/p3at_controller.cc)
target_link_libraries(p3at ${catkin_LIBRARIES} tinyxml2)

##add_executable(p3at src/p3at.cc)

##target_link_libraries(p3at ${catkin_LIBRARIES})

install(TARGETS p3at
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
