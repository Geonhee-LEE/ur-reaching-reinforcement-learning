cmake_minimum_required(VERSION 2.8.3)
project(ur_robotiq_moveit_api)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
    COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ur_move_group_interface
    CATKIN_DEPENDS
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
      tf2_geometry_msgs
    DEPENDS
      EIGEN3
      )


include_directories(include ${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(ur_move_group_interface
  src/ur_move_group_interface.cpp
)

target_link_libraries(ur_move_group_interface
  ${interactive_markers_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
