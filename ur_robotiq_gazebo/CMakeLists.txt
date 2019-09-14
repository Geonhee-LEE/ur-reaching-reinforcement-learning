cmake_minimum_required(VERSION 2.8.3)
project(ur_robotiq_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED   gazebo_msgs roscpp geometry_msgs std_srvs std_msgs message_generation ur_rl_msgs )

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES blocks_poses_publisher
  CATKIN_DEPENDS  ur_rl_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

#### Multiple blocks
# Declare the executable, along with its source files.
add_executable(blocks_spawner src/blocks_spawner.cpp)
# Specify libraries against which to link.
target_link_libraries(blocks_spawner ${catkin_LIBRARIES})

add_executable(blocks_poses_publisher src/blocks_poses_publisher.cpp)
target_link_libraries(blocks_poses_publisher ${catkin_LIBRARIES})
add_dependencies(blocks_poses_publisher ${catkin_EXPORTED_TARGETS})


#### A block
# Declare the executable, along with its source files.
add_executable(block_spawner src/block_spawner.cpp)
# Specify libraries against which to link.
target_link_libraries(block_spawner ${catkin_LIBRARIES})

add_executable(block_pose_publisher src/block_pose_publisher.cpp)
target_link_libraries(block_pose_publisher ${catkin_LIBRARIES})
add_dependencies(block_pose_publisher ${catkin_EXPORTED_TARGETS})
#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ur_gazebo ur_gazebo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
