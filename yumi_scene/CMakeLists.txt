cmake_minimum_required(VERSION 3.1.3)
project(yumi_scene)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  interactive_markers
  moveit_core
  moveit_commander
)

 
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    rospy
    std_msgs
    moveit_core
    moveit_commander
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/yumi_scene.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
