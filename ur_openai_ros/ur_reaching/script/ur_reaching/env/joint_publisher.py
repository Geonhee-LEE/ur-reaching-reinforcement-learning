#!/usr/bin/env python

import rospy
import math
import time
import copy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from .controllers_connection import ControllersConnection

class JointPub(object):
    def __init__(self):
        # Controller type for ros_control
        self.current_controller_type =  rospy.get_param("/control_type")
        print ("###### init #####, self.current_controller_type: ", self.current_controller_type)
        self._ctrl_conn = ControllersConnection(namespace="")

        if self.current_controller_type == "pos" or self.current_controller_type == "traj_pos":
    	    print ("###### init #####, load_pos_controllers ")			
    	    self._ctrl_conn.load_controllers("ur_shoulder_pan_pos_controller")
    	    self._ctrl_conn.load_controllers("ur_shoulder_lift_pos_controller")
    	    self._ctrl_conn.load_controllers("ur_elbow_pos_controller")
    	    self._ctrl_conn.load_controllers("ur_wrist_1_pos_controller")
    	    self._ctrl_conn.load_controllers("ur_wrist_2_pos_controller")
    	    self._ctrl_conn.load_controllers("ur_wrist_3_pos_controller")

    	    self._shoulder_pan_joint_pub = rospy.Publisher('/ur_shoulder_pan_pos_controller/command', Float64, queue_size=1)
    	    self._shoulder_lift_joint_pub = rospy.Publisher('/ur_shoulder_lift_pos_controller/command', Float64, queue_size=1)
    	    self._elbow_joint_pub = rospy.Publisher('/ur_elbow_pos_controller/command', Float64, queue_size=1)
    	    self._wrist_1_joint_pub = rospy.Publisher('/ur_wrist_1_pos_controller/command', Float64, queue_size=1)
    	    self._wrist_2_joint_pub = rospy.Publisher('/ur_wrist_2_pos_controller/command', Float64, queue_size=1)
    	    self._wrist_3_joint_pub = rospy.Publisher('/ur_wrist_3_pos_controller/command', Float64, queue_size=1)
        elif self.current_controller_type == "vel" or self.current_controller_type == "traj_vel":
    	    print ("###### init #####, load_vel_controllers ")		
    	    self._ctrl_conn.load_controllers("ur_shoulder_pan_vel_controller")
    	    self._ctrl_conn.load_controllers("ur_shoulder_lift_vel_controller")
    	    self._ctrl_conn.load_controllers("ur_elbow_vel_controller")
    	    self._ctrl_conn.load_controllers("ur_wrist_1_vel_controller")
    	    self._ctrl_conn.load_controllers("ur_wrist_2_vel_controller")
    	    self._ctrl_conn.load_controllers("ur_wrist_3_vel_controller")

    	    self._shoulder_pan_joint_pub = rospy.Publisher('/ur_shoulder_pan_vel_controller/command', Float64, queue_size=1)
    	    self._shoulder_lift_joint_pub = rospy.Publisher('/ur_shoulder_lift_vel_controller/command', Float64, queue_size=1)
    	    self._elbow_joint_pub = rospy.Publisher('/ur_elbow_vel_controller/command', Float64, queue_size=1)
    	    self._wrist_1_joint_pub = rospy.Publisher('/ur_wrist_1_vel_controller/command', Float64, queue_size=1)
    	    self._wrist_2_joint_pub = rospy.Publisher('/ur_wrist_2_vel_controller/command', Float64, queue_size=1)
    	    self._wrist_3_joint_pub = rospy.Publisher('/ur_wrist_3_vel_controller/command', Float64, queue_size=1)
        else:
    	    print ("Fail to load controllers")		

        self.publishers_array = []
        self.publishers_array.append(self._shoulder_pan_joint_pub)
        self.publishers_array.append(self._shoulder_lift_joint_pub)
        self.publishers_array.append(self._elbow_joint_pub)
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
    	rate = rospy.Rate(1)  # 1hz

    	#print ("self.current_controller_type: ", self.current_controller_type)
    	if self.current_controller_type == "pos":
    	    while (self._shoulder_pan_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _shoulder_pan_joint_pub yet so we wait and try again")
    	        try:
    	        	#print ("start on ur_shoulder_pan_pos_controller")
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_shoulder_pan_pos_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_shoulder_pan_joint_pub Publisher Connected")

    	    while (self._shoulder_lift_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _shoulder_lift_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_shoulder_lift_pos_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_shoulder_lift_joint_pub Publisher Connected")

    	    while (self._elbow_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _elbow_pos_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_elbow_pos_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_elbow_joint_pub Publisher Connected")

    	    while (self._wrist_1_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _wrist_1_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_wrist_1_pos_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_wrist_1_joint_pub Publisher Connected")

    	    while (self._wrist_2_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _wrist_2_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_wrist_2_pos_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_wrist_2_joint_pub Publisher Connected")

    	    while (self._wrist_3_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _wrist_3_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_wrist_3_pos_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_wrist_3_joint_pub Publisher Connected")

    	    rospy.logdebug("All Joint Publishers READY")
    	elif self.current_controller_type == "vel":
    	    while (self._shoulder_pan_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _shoulder_pan_joint_pub yet so we wait and try again")
    	        try:
    	        	#print ("start on ur_shoulder_pan_vel_controller")
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_shoulder_pan_vel_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_shoulder_pan_joint_pub Publisher Connected")

    	    while (self._shoulder_lift_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _shoulder_lift_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_shoulder_lift_vel_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_shoulder_lift_joint_pub Publisher Connected")

    	    while (self._elbow_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _elbow_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_elbow_vel_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_elbow_joint_pub Publisher Connected")

    	    while (self._wrist_1_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _wrist_1_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_wrist_1_vel_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_wrist_1_joint_pub Publisher Connected")


    	    while (self._wrist_2_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _wrist_2_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_wrist_2_vel_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_wrist_2_joint_pub Publisher Connected")


    	    while (self._wrist_3_joint_pub.get_num_connections() == 0):
    	        rospy.logdebug("No susbribers to _wrist_3_joint_pub yet so we wait and try again")
    	        try:
    	        	self._ctrl_conn.start_controllers(controllers_on="ur_wrist_3_vel_controller")
    	        	rate.sleep()
    	        except rospy.ROSInterruptException:
    	        	# This is to avoid error when world is rested, time when backwards.
    	        	pass
    	    rospy.logdebug("_wrist_3_joint_pub Publisher Connected")

    	    rospy.logdebug("All Joint Publishers READY")

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
