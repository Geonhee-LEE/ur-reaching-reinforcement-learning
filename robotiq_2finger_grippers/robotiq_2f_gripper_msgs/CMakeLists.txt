cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_gripper_msgs)

find_package(catkin 
    REQUIRED genmsg actionlib actionlib_msgs 
    COMPONENTS message_generation std_msgs)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
    DIRECTORY 
        msg 
    FILES 
        RobotiqGripperCommand.msg 
        RobotiqGripperStatus.msg
)

add_action_files(
  DIRECTORY 
    action
  FILES 
    CommandRobotiqGripper.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs)

                      
