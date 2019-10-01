cmake_minimum_required(VERSION 2.8.3)
project(ur_reinforcement_learning)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
scripts/ur_reinforcement_learning/ppo_gae_main.py
scripts/ur_reinforcement_learning/reinforcement_main.py
scripts/ur_reinforcement_learning/gazebo_execution.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


