cmake_minimum_required(VERSION 2.8.3)
project(ur_reaching)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
scripts/ur_reaching/reaching_main.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


