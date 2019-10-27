#! /usr/bin/env python
"""--------------------------------------------------------------------
This Node/Script shows an example on how to use a `SimpleActionClient` instance 
to control a Robotiq 2 Finger gripper.

Parameters:
    action_name: Name of the action advertised by the `ActionServer` controlling the gripper.

@author: Daniel Felipe Ordonez Apraez
@email: daniels.ordonez@gmail.com
--------------------------------------------------------------------"""

import rospy

# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

def operate_gripper():

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    # Wait until grippers are ready to take command
    robotiq_client.wait_for_server()

    rospy.logwarn("Client test: Starting sending goals")
    ## Manually set all the parameters of the gripper goal state.
    ######################################################################################

    goal = CommandRobotiqGripperGoal()
    goal.emergency_release = False
    goal.stop = False
    goal.position = 0.00
    goal.speed = 0.1
    goal.force = 5.0

    # Sends the goal to the gripper.
    robotiq_client.send_goal(goal)
    # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
    robotiq_client.wait_for_result()

    # Use pre-defined functions for robot gripper manipulation.
    #####################################################################################
    while not rospy.is_shutdown():
        Robotiq.goto(robotiq_client, pos=0.00, speed=0.1, force=100 , block=True)
        Robotiq.goto(robotiq_client, pos=0.04, speed=0.01, force=10)
        Robotiq.goto(robotiq_client, pos=0.011, speed=0.01, force=0 , block=True)
        Robotiq.goto(robotiq_client, pos=0.08, speed=0.11, force=200 , block=True)
        # Robotiq.goto(robotiq_client, pos=0.06, speed=0.0, force=0)
        # break

    # Prints out the result of executing the action
    return robotiq_client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('robotiq_2f_client')
    result = operate_gripper()