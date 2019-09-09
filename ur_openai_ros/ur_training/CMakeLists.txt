cmake_minimum_required(VERSION 2.8.3)
project(ur_training)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
scripts/ur_training/start_training_v2.py
scripts/ur_training/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


