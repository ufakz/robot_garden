cmake_minimum_required(VERSION 2.8.3)
project(p3at_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/spawn_robot.py  # Make sure the script is placed inside the 'scripts' folder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf launch meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
