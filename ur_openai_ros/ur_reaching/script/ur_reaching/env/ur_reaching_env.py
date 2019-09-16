#!/usr/bin/env python
'''
    By Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import sys
import time

import gym
import rospy
import numpy as np

from gym import utils, spaces
from geometry_msgs.msg import Pose
from gym.utils import seeding
# For register my env
from gym.envs.registration import register

# For inherit RobotGazeboEnv
from ur_reaching.env import robot_gazebo_env_goal
# About reset GAZEBO simultor
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from joint_traj_publisher import JointTrajPub
from ur_state import URState
from controllers_connection import ControllersConnection
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState
from gazebo_msgs.srv import GetWorldProperties

rospy.loginfo("register...")
#register the training environment in the gym as an available one
reg = gym.envs.register(
    id='URReaching-v0',
    entry_point='ur_reaching.env.ur_reaching_env:URReaching', # Its directory associated with importing in other sources like from 'ur_reaching.env.ur_sim_env import *' 
    #timestep_limit=100000,
    )

class URReaching(robot_gazebo_env_goal.RobotGazeboEnv):
    def __init__(self):
    	# We assume that a ROS node has already been created before initialising the environment
        # Init GAZEBO Objects
        self.set_obj_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_world_state = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

    	# Gets training parameters from param server
    	self.desired_pose = Pose()
    	self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
    	self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
    	self.desired_pose.position.z = rospy.get_param("/desired_pose/z")
    	self.running_step = rospy.get_param("/running_step")
    	self.max_incl = rospy.get_param("/max_incl")
    	self.max_height = rospy.get_param("/max_height")
    	self.min_height = rospy.get_param("/min_height")
    	self.joint_increment_value = rospy.get_param("/joint_increment_value")
    	self.done_reward = rospy.get_param("/done_reward")
    	self.alive_reward = rospy.get_param("/alive_reward")
    	self.desired_force = rospy.get_param("/desired_force")
    	self.desired_yaw = rospy.get_param("/desired_yaw")

    	self.list_of_observations = rospy.get_param("/list_of_observations")
    	
    	shp_max = rospy.get_param("/joint_limits_array/shp_max")
    	shp_min = rospy.get_param("/joint_limits_array/shp_min")
    	shl_max = rospy.get_param("/joint_limits_array/shl_max")
    	shl_min = rospy.get_param("/joint_limits_array/shl_min")
    	elb_max = rospy.get_param("/joint_limits_array/elb_max")
    	elb_min = rospy.get_param("/joint_limits_array/elb_min")
    	wr1_max = rospy.get_param("/joint_limits_array/wr1_max")
    	wr1_min = rospy.get_param("/joint_limits_array/wr1_min")
    	wr2_max = rospy.get_param("/joint_limits_array/wr2_max")
    	wr2_min = rospy.get_param("/joint_limits_array/wr2_min")
    	wr3_max = rospy.get_param("/joint_limits_array/wr3_max")
    	wr3_min = rospy.get_param("/joint_limits_array/wr3_min")
    	self.joint_limits = {"shp_max": shp_max,
    	    	    	     "shp_min": shp_min,
    	    	    	     "shl_max": shl_max,
    	    	    	     "shl_min": shl_min,
    	    	    	     "elb_max": elb_max,
    	    	    	     "elb_min": elb_min,
    	    	    	     "wr1_max": wr1_max,
    	    	    	     "wr1_min": wr1_min,
    	    	    	     "wr2_max": wr2_max,
    	    	    	     "wr2_min": wr2_min,
    	    	    	     "wr3_max": wr3_max,
    	    	    	     "wr3_min": wr3_min
    	    	    	     }

    	self.discrete_division = rospy.get_param("/discrete_division")

    	self.maximum_base_linear_acceleration = rospy.get_param("/maximum_base_linear_acceleration")
    	self.maximum_base_angular_velocity = rospy.get_param("/maximum_base_angular_velocity")
    	self.maximum_joint_effort = rospy.get_param("/maximum_joint_effort")

    	self.weight_r1 = rospy.get_param("/weight_r1")
    	self.weight_r2 = rospy.get_param("/weight_r2")
    	self.weight_r3 = rospy.get_param("/weight_r3")
    	self.weight_r4 = rospy.get_param("/weight_r4")
    	self.weight_r5 = rospy.get_param("/weight_r5")

    	shp_init_value = rospy.get_param("/init_joint_pose/shp")
    	shl_init_value = rospy.get_param("/init_joint_pose/shl")
    	elb_init_value = rospy.get_param("/init_joint_pose/elb")
    	wr1_init_value = rospy.get_param("/init_joint_pose/wr1")
    	wr2_init_value = rospy.get_param("/init_joint_pose/wr2")
    	wr3_init_value = rospy.get_param("/init_joint_pose/wr3")
    	self.init_joint_pose = [shp_init_value, shl_init_value, elb_init_value, wr1_init_value, wr2_init_value, wr3_init_value]

    	# Fill in the Done Episode Criteria list
    	self.episode_done_criteria = rospy.get_param("/episode_done_criteria")
    	
    	# stablishes connection with simulator
    	self._gz_conn = GazeboConnection()
    	self._ctrl_conn = ControllersConnection(namespace="")
		
    	self._ctrl_type =  sys.argv[1]
    	self.pre_ctrl_type =  self._ctrl_type

    	self._ur_state = URState(   max_height=self.max_height,
    	    	    	    	    min_height=self.min_height,
    	    	    	    	    abs_max_roll=self.max_incl,
    	    	    	    	    abs_max_pitch=self.max_incl,
    	    	    	    	    joint_increment_value=self.joint_increment_value,
    	    	    	    	    list_of_observations=self.list_of_observations,
    	    	    	    	    joint_limits=self.joint_limits,
    	    	    	    	    episode_done_criteria=self.episode_done_criteria,
    	    	    	    	    done_reward=self.done_reward,
    	    	    	    	    alive_reward=self.alive_reward,
    	    	    	    	    desired_force=self.desired_force,
    	    	    	    	    desired_yaw=self.desired_yaw,
    	    	    	    	    weight_r1=self.weight_r1,
    	    	    	    	    weight_r2=self.weight_r2,
    	    	    	    	    weight_r3=self.weight_r3,
    	    	    	    	    weight_r4=self.weight_r4,
    	    	    	    	    weight_r5=self.weight_r5,
    	    	    	    	    discrete_division=self.discrete_division,
    	    	    	    	    maximum_base_linear_acceleration=self.maximum_base_linear_acceleration,
    	    	    	    	    maximum_base_angular_velocity=self.maximum_base_angular_velocity,
    	    	    	    	    maximum_joint_effort=self.maximum_joint_effort
    	    	    	    	)

    	self._ur_state.set_desired_world_point( self.desired_pose.position.x,
    	    	    	    	    	    	self.desired_pose.position.y,
    	    	    	    	    	    	self.desired_pose.position.z)

    	self._joint_pubisher = JointPub()
    	self._joint_traj_pubisher = JointTrajPub()
		
		# Switch flag: if controller is switched, publisher connection occurs error without reset
    	self.switch_flag = True 
    	
    	"""
    	For this version, we consider 6 actions
    	1) Increment/Decrement shp_joint_value
    	2) Increment/Decrement shl_joint_value
    	3) Increment/Decrement elb_joint_value
    	4) Increment/Decrement wr1_joint_value
    	5) Increment/Decrement wr2_joint_value
    	6) Increment/Decrement wr3_joint_value
    	"""
        # Gym interface
    	self.action_space = spaces.Discrete(6)
    	self.reward_range = (-np.inf, np.inf)
    	self._seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
    	self.np_random, seed = seeding.np_random(seed)
    	return [seed]
    	
    # Resets the state of the environment and returns an initial observation.
    def reset(self):
    	# 0st: We pause the Simulator
    	rospy.logdebug("Pausing SIM...")
    	self._gz_conn.pauseSim()

    	# 1st: resets the simulation to initial values
    	rospy.logdebug("Reset SIM...")
    	self._gz_conn.resetSim()

    	# 2nd: We Set the gravity to 0.0 so that we dont fall when reseting joints
    	# It also UNPAUSES the simulation
    	rospy.logdebug("Remove Gravity...")
    	self._gz_conn.change_gravity(0.0, 0.0, 0.0)

    	# EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
    	rospy.logdebug("reset_ur_joint_controllers...")
    	if self._ctrl_type != self.pre_ctrl_type:
    		print('self._ctrl_type != self.pre_ctrl_type', self._ctrl_type, self.pre_ctrl_type)
    		if self._ctrl_type == 'vel':
    			self._ctrl_conn.switch_controllers(controllers_on= self._ctrl_conn.vel_controller, 
									controllers_off = self._ctrl_conn.vel_traj_controller)
    		elif self._ctrl_type == 'traj_vel':
    			self._ctrl_conn.switch_controllers(controllers_off= self._ctrl_conn.vel_controller, 
									controllers_on = self._ctrl_conn.vel_traj_controller)
    	else:
    		self._ctrl_conn.reset_ur_joint_controllers(self._ctrl_type)

    	# 3rd: resets the robot to initial conditions
    	rospy.logdebug("set_init_pose init variable...>>>" + str(self.init_joint_pose))
    	# We save that position as the current joint desired position
    	init_pos = self._ur_state.init_joints_pose(self.init_joint_pose)
        print("init_pos")

    	# 4th: We Set the init pose to the jump topic so that the jump control can update
    	# We check the jump publisher has connection

    	if self._ctrl_type == 'traj_vel':
        	self._joint_traj_pubisher.check_publishers_connection()
    	elif self._ctrl_type == 'vel':
    		self._joint_pubisher.check_publishers_connection()
    	else:
        	rospy.logwarn("Controller type is wrong!!!!")
        print("check_publishers_connection ")
    	

    	# 5th: Check all subscribers work.
    	# Get the state of the Robot defined by its RPY orientation, distance from
    	# desired point, contact force and JointState of the three joints
    	rospy.logdebug("check_all_systems_ready...")
    	self._ur_state.check_all_systems_ready()

    	# 6th: We restore the gravity to original
    	rospy.logdebug("Restore Gravity...")
    	self._gz_conn.change_gravity(0.0, 0.0, -9.81)

    	# 7th: pauses simulation
    	rospy.logdebug("Pause SIM...")
    	self._gz_conn.pauseSim()
        # self._init_obj_pose()

    	# 8th: Get the State Discrete Stringuified version of the observations
    	rospy.logdebug("get_observations...")
    	observation = self._ur_state.get_observations()
    	state = self.get_state(observation)

        print('Reset final')
    	return state

    def _act(self, action):
    	if self._ctrl_type == 'traj_vel':
    		self.pre_ctrl_type = 'traj_vel'
    		self._joint_traj_pubisher.move_joints(action)
    	elif self._ctrl_type == 'vel':
        	self.pre_ctrl_type = 'vel'
    		self._joint_pubisher.move_joints(action)
    	else:
    		self._joint_pubisher.move_joints(action)
				
    def step(self, action):
    	rospy.logdebug("UR step func")

    	# Given the action selected by the learning algorithm,
    	# we perform the corresponding movement of the robot

    	# 1st, decide which action corresponds to which joint is incremented
    	next_action_position = self._ur_state.get_action_to_position(action)

    	# We move it to that pos
    	self._gz_conn.unpauseSim()
    	self._act(next_action_position)
    	
    	# Then we send the command to the robot and let it go
    	# for running_step seconds
    	time.sleep(self.running_step)
    	self._gz_conn.pauseSim()

    	# We now process the latest data saved in the class state to calculate
    	# the state and the rewards. This way we guarantee that they work
    	# with the same exact data.
    	# Generate State based on observations
    	observation = self._ur_state.get_observations()

    	# finally we get an evaluation based on what happened in the sim
    	reward,done = self._ur_state.process_data()

    	# Get the State Discrete Stringuified version of the observations
    	state = self.get_state(observation)

    	return state, reward, done, {}

    def get_state(self, observation):
    	"""
    	We retrieve the Stringuified-Discrete version of the given observation
    	:return: state
    	"""
    	return self._ur_state.get_state_as_string(observation)

    def _init_obj_pose(self):
        """Inits object pose for arrangement at reset time
        _init_obj_pose() isn't completely worked, but if loaded each sdf file follwing a object it can be worked.
        """
        x = [0]
        y = [0]
        z = [0]

        qx = [0]
        qy = [0]
        qz = [0]
        qw = [1]

        req_position = np.array([x, y, z])
        req_orientation = np.array([qx, qy, qz, qw])
        req_name = 'dropbox'
        while not self._set_obj_position(req_name, req_position, req_orientation):
            pass
        req_name = 'dropbox_clone'
        while not self._set_obj_position(req_name, req_position, req_orientation):
            pass
        req_name = 'short_table'
        while not self._set_obj_position(req_name, req_position, req_orientation):
            pass
        req_name = 'short_table_clone'
        while not self._set_obj_position(req_name, req_position, req_orientation):
            pass

    def _set_obj_position(self, obj_name, position, orientation):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo("set model_state for " + obj_name + " available")
        sms_req = SetModelStateRequest()
        sms_req.model_state.pose.position.x = position[0]
        sms_req.model_state.pose.position.y = position[1]
        sms_req.model_state.pose.position.z = position[2]
        sms_req.model_state.pose.orientation.x = orientation[0]
        sms_req.model_state.pose.orientation.y = orientation[1]
        sms_req.model_state.pose.orientation.z = orientation[2]
        sms_req.model_state.pose.orientation.w = orientation[3]

        sms_req.model_state.twist.linear.x = 0.
        sms_req.model_state.twist.linear.y = 0.
        sms_req.model_state.twist.linear.z = 0.
        sms_req.model_state.twist.angular.x = 0.
        sms_req.model_state.twist.angular.y = 0.
        sms_req.model_state.twist.angular.z = 0.
        sms_req.model_state.model_name = obj_name
        sms_req.model_state.reference_frame = 'ground_plane'
        result = self.set_obj_state(sms_req)

        rospy.loginfo("result.success: " + str(result.success) )
        return result.success