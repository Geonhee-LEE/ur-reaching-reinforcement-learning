#!/usr/bin/env python
'''
    By Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import gym
import rospy
import numpy as np
import time

from gym import utils, spaces
from geometry_msgs.msg import Pose
from gym.utils import seeding
# For register my env
from gym.envs.registration import register

# For inherit RobotGazeboEnv
from ur_training.env import robot_gazebo_env_goal
# About reset GAZEBO simultor
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from ur_state import URState
from controllers_connection import ControllersConnection

rospy.loginfo("register...")
#register the training environment in the gym as an available one
reg = gym.envs.register(
    id='URTest-v0',
    entry_point='ur_training.env.ur_test_env:URTestEnv', # Its directory associated with importing in other sources like from 'ur_training.env.ur_sim_env import *' 
    #timestep_limit=100000,
    )

class URTestEnv(robot_gazebo_env_goal.RobotGazeboEnv):
    def __init__(self):
    	# We assume that a ROS node has already been created before initialising the environment

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
    	self._ctrl_conn.reset_ur_joint_controllers()

    	# 3rd: resets the robot to initial conditions
    	rospy.logdebug("set_init_pose init variable...>>>" + str(self.init_joint_pose))
    	# We save that position as the current joint desired position
    	init_pos = self._ur_state.init_joints_pose(self.init_joint_pose)

    	# 4th: We Set the init pose to the jump topic so that the jump control can update
    	rospy.logdebug("Publish init_pose for Jump Control...>>>" + str(init_pos))
    	# We check the jump publisher has connection
    	self._joint_pubisher.check_publishers_connection()
    	# We move the joints to position
    	self._joint_pubisher.move_joints(init_pos)

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

    	# 8th: Get the State Discrete Stringuified version of the observations
    	rospy.logdebug("get_observations...")
    	observation = self._ur_state.get_observations()
    	state = self.get_state(observation)

    	return state

    def step(self, action):
    	rospy.logdebug("URTest step func")

    	# Given the action selected by the learning algorithm,
    	# we perform the corresponding movement of the robot

    	# 1st, decide which action corresponds to which joint is incremented
    	next_action_position = self._ur_state.get_action_to_position(action)

    	# We move it to that pos
    	self._gz_conn.unpauseSim()
    	self._joint_pubisher.move_joints(next_action_position)
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
