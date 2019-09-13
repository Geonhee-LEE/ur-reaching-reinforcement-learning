#!/usr/bin/env python

import rospy
import math
import time
import copy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self._shoulder_pan_joint_pub = rospy.Publisher('/ur_shoulder_pan_vel_controller/command', Float64, queue_size=1)
        self._shoulder_lift_joint_pub = rospy.Publisher('/ur_shoulder_lift_vel_controller/command', Float64, queue_size=1)
        self._elbow_vel_joint_pub = rospy.Publisher('/ur_elbow_vel_controller/command', Float64, queue_size=1)
        self._wrist_1_joint_pub = rospy.Publisher('/ur_wrist_1_vel_controller/command', Float64, queue_size=1)
        self._wrist_2_joint_pub = rospy.Publisher('/ur_wrist_2_vel_controller/command', Float64, queue_size=1)
        self._wrist_3_joint_pub = rospy.Publisher('/ur_wrist_3_vel_controller/command', Float64, queue_size=1)
        
        self.publishers_array.append(self._shoulder_pan_joint_pub)
        self.publishers_array.append(self._shoulder_lift_joint_pub)
        self.publishers_array.append(self._elbow_vel_joint_pub)
        self.publishers_array.append(self._wrist_1_joint_pub)
        self.publishers_array.append(self._wrist_2_joint_pub)
        self.publishers_array.append(self._wrist_3_joint_pub)


    def set_init_pose(self, init_pose):
        """
        Sets joints to initial position [0,0,0]
        :return: The init Pose
        """
        self.check_publishers_connection()
        self.move_joints(init_pose)

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._shoulder_pan_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _shoulder_pan_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_shoulder_pan_joint_pub Publisher Connected")

        while (self._shoulder_lift_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _shoulder_lift_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_shoulder_lift_joint_pub Publisher Connected")

        while (self._elbow_vel_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _elbow_vel_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_elbow_vel_joint_pub Publisher Connected")


        while (self._wrist_1_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _wrist_1_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_wrist_1_joint_pub Publisher Connected")


        while (self._wrist_2_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _wrist_2_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_wrist_2_joint_pub Publisher Connected")


        while (self._wrist_3_joint_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _wrist_3_joint_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_wrist_3_joint_pub Publisher Connected")

        rospy.logdebug("All Joint Publishers READY")

    def joint_mono_des_callback(self, msg):
        rospy.logdebug(str(msg.joint_state.position))

        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.logdebug("JointsPos>>"+str(joint_value))
          publisher_object.publish(joint_value)
          i += 1

    def start_loop(self, rate_value = 2.0):
        rospy.logdebug("Start Loop")
        pos1 = [-2.873, 0.0, 6.28, 3.015, 1.21, 1.264, -0.97]
        pos2 = [-2.873, 0.0, 6.28, 3.015, 1.21, 1.264, 0.97]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
          if position == "pos1":
            self.move_joints(pos1)
            position = "pos2"
          else:
            self.move_joints(pos2)
            position = "pos1"
          rate.sleep()

    def start_sinus_loop(self, rate_value = 2.0):
        rospy.logdebug("Start Loop")
        w = 0.0
        x = 1.0*math.sin(w)
        #pos_x = [0.0,0.0,x]
        pos_x = [x, 0.0, 0.0]
        #pos_x = [0.0, x, 0.0]
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            self.move_joints(pos_x)
            w += 0.05
            x = 1.0 * math.sin(w)
            #pos_x = [0.0, 0.0, x]
            pos_x = [x, 0.0, 0.0]
            #pos_x = [0.0, x, 0.0]
            print (x)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node('joint_publisher_node', log_level=rospy.WARN)
    joint_publisher = JointPub()
    rate_value = 8.0
    #joint_publisher.start_loop(rate_value)
    #joint_publisher.start_sinus_loop(rate_value)
