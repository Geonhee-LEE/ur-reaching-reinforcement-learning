#!/usr/bin/env python

import rospy
import math
import time
import copy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from .controllers_connection import ControllersConnection

class JointTrajPub(object):
    def __init__(self):
        """
        Publish trajectory_msgs::JointTrajectory for velocity control
        """
        self._ctrl_conn = ControllersConnection(namespace="")
        current_controller_type =  rospy.get_param("/control_type")

        if (current_controller_type == "pos") or (current_controller_type == "traj_pos"):
        	self._ctrl_conn.load_controllers("pos_traj_controller")
        	self._joint_traj_pub = rospy.Publisher('/pos_traj_controller/command', JointTrajectory, queue_size=10)
        else:
        	self._ctrl_conn.load_controllers("vel_traj_controller")
        	self._joint_traj_pub = rospy.Publisher('/vel_traj_controller/command', JointTrajectory, queue_size=10)

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
    	while (self._joint_traj_pub.get_num_connections() == 0):
    	    rospy.logdebug("No subscribers to vel_traj_controller yet so we wait and try again")
    	    try:
    	    	self._ctrl_conn.start_controllers(controllers_on="vel_traj_controller")
    	    	rate.sleep()
    	    except rospy.ROSInterruptException:
    	    	# This is to avoid error when world is rested, time when backwards.
    	    	pass
    	rospy.logdebug("_joint_traj_pub Publisher Connected")

    	rospy.logdebug("All Joint Publishers READY")

    def move_joints(self, joints_array):
    	vel_cmd = JointTrajectory()
    	vel_cmd.joint_names = [ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" ]
    	vel_cmd.header.stamp = rospy.Time.now()
    	# create a JTP instance and configure it
    	jtp = JointTrajectoryPoint(positions=[0]*6, velocities=[0]*6 , time_from_start=rospy.Duration(.0))
    	jtp.velocities = [joints_array[0], joints_array[1], joints_array[2], joints_array[3], joints_array[4], joints_array[5] ]
    	
    	# setup the reset of the pt
    	vel_cmd.points =[jtp]
    	self._joint_traj_pub.publish(vel_cmd)

    def jointTrajectoryCommand(self, joints_array):
    	rospy.loginfo("jointTrajectoryCommand")
    	try:    
    	    rospy.loginfo (rospy.get_rostime().to_sec())
    	    while rospy.get_rostime().to_sec() == 0.0:
    	    	time.sleep(0.1)
    	    	rospy.loginfo (rospy.get_rostime().to_sec())

    	    jt = JointTrajectory()
    	    jt.header.stamp = rospy.Time.now()
    	    jt.header.frame_id = "ur5"
    	    jt.joint_names.append("shoulder_pan_joint")
    	    jt.joint_names.append("shoulder_lift_joint")
    	    jt.joint_names.append("elbow_joint")
    	    jt.joint_names.append("wrist_1_joint")
    	    jt.joint_names.append("wrist_2_joint")
    	    jt.joint_names.append("wrist_3_joint")
    	    	    
    	    dt = 0.01
    	    p = JointTrajectoryPoint()
    	    p.positions.append(joints_array[0])
    	    p.positions.append(joints_array[1])
    	    p.positions.append(joints_array[2])
    	    p.positions.append(joints_array[3])
    	    p.positions.append(joints_array[4])
    	    p.positions.append(joints_array[5])
    	    jt.points.append(p)

    	    # set duration
    	    jt.points[0].time_from_start = rospy.Duration.from_sec(dt)
    	    rospy.loginfo("Test: velocities")
    	    self._joint_traj_pub.publish(jt)

    	except rospy.ROSInterruptException: pass

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
    joint_publisher = JointTrajPub()
    rate_value = 8.0
    #joint_publisher.start_loop(rate_value)
    #joint_publisher.start_sinus_loop(rate_value)
