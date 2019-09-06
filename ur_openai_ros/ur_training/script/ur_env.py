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
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from ur_state import URState
from controllers_connection import ControllersConnection


rospy.loginfo("register...")
#register the training environment in the gym as an available one
reg = register(
    id='UR-v0',
    entry_point='ur_env:UREnv',
    #timestep_limit=100000,
    )

class UREnv(gym.Env):
    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment

        # gets training parameters from param server
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

        # self.list_of_observations = rospy.get_param("/list_of_observations")

        haa_max = rospy.get_param("/joint_limits_array/haa_max")
        haa_min = rospy.get_param("/joint_limits_array/haa_min")
        hfe_max = rospy.get_param("/joint_limits_array/hfe_max")
        hfe_min = rospy.get_param("/joint_limits_array/hfe_min")
        kfe_max = rospy.get_param("/joint_limits_array/kfe_max")
        kfe_min = rospy.get_param("/joint_limits_array/kfe_min")
        self.joint_limits = {"haa_max": haa_max,
                             "haa_min": haa_min,
                             "hfe_max": hfe_max,
                             "hfe_min": hfe_min,
                             "kfe_max": kfe_max,
                             "kfe_min": kfe_min
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

        haa_init_value = rospy.get_param("/init_joint_pose/haa")
        hfe_init_value = rospy.get_param("/init_joint_pose/hfe")
        kfe_init_value = rospy.get_param("/init_joint_pose/kfe")
        self.init_joint_pose = [haa_init_value,hfe_init_value,kfe_init_value]

        # Fill in the Done Episode Criteria list
        self.episode_done_criteria = rospy.get_param("/episode_done_criteria")

        # Jump Increment Value in Radians
        self.jump_increment = rospy.get_param("/jump_increment")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.controllers_object = ControllersConnection(namespace="ur")

        self.ur_state_object = URState(   max_height=self.max_height,
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
                                                    maximum_joint_effort=self.maximum_joint_effort,
                                                    jump_increment=self.jump_increment
                                                )

        self.ur_state_object.set_desired_world_point(self.desired_pose.position.x,
                                                          self.desired_pose.position.y,
                                                          self.desired_pose.position.z)

        self.ur_joint_pubisher_object = JointPub()
        


        """
        For this version, we consider 5 actions
        1-2) Increment/Decrement haa_joint
        3-4) Increment/Decrement hfe_joint
        5) Dont Move
        6) Perform One Jump
        """
        self.action_space = spaces.Discrete(6)
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def _reset(self):

        # 0st: We pause the Simulator
        rospy.logdebug("Pausing SIM...")
        self.gazebo.pauseSim()

        # 1st: resets the simulation to initial values
        rospy.logdebug("Reset SIM...")
        self.gazebo.resetSim()

        # 2nd: We Set the gravity to 0.0 so that we dont fall when reseting joints
        # It also UNPAUSES the simulation
        rospy.logdebug("Remove Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        rospy.logdebug("reset_ur_joint_controllers...")
        self.controllers_object.reset_ur_joint_controllers()

        # 3rd: resets the robot to initial conditions
        rospy.logdebug("set_init_pose init variable...>>>" + str(self.init_joint_pose))
        # We save that position as the current joint desired position
        init_pos = self.ur_state_object.init_joints_pose(self.init_joint_pose)

        # 4th: We Set the init pose to the jump topic so that the jump control can update
        rospy.logdebug("Publish init_pose for Jump Control...>>>" + str(init_pos))
        # We check the jump publisher has connection
        self.ur_joint_pubisher_object.check_publishers_connection()
        # We move the joints to position, no jump
        do_jump = False
        self.ur_joint_pubisher_object.move_joints_jump(init_pos, do_jump)

        # 5th: Check all subscribers work.
        # Get the state of the Robot defined by its RPY orientation, distance from
        # desired point, contact force and JointState of the three joints
        rospy.logdebug("check_all_systems_ready...")
        self.ur_state_object.check_all_systems_ready()

        # 6th: We restore the gravity to original
        rospy.logdebug("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # 8th: Get the State Discrete Stringuified version of the observations
        rospy.logdebug("get_observations...")
        observation = self.ur_state_object.get_observations()
        state = self.get_state(observation)

        return state

    def _step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot

        # 1st, decide which action corresponds to which joint is incremented
        next_action_position, do_jump = self.ur_state_object.get_action_to_position(action)

        # We move it to that pos
        self.gazebo.unpauseSim()
        self.ur_joint_pubisher_object.move_joints_jump(next_action_position, do_jump)
        # Then we send the command to the robot and let it go
        # for running_step seconds
        time.sleep(self.running_step)
        self.gazebo.pauseSim()

        # We now process the latest data saved in the class state to calculate
        # the state and the rewards. This way we guarantee that they work
        # with the same exact data.
        # Generate State based on observations
        observation = self.ur_state_object.get_observations()

        # finally we get an evaluation based on what happened in the sim
        reward,done = self.ur_state_object.process_data()

        # Get the State Discrete Stringuified version of the observations
        state = self.get_state(observation)

        return state, reward, done, {}

    def get_state(self, observation):
        """
        We retrieve the Stringuified-Discrete version of the given observation
        :return: state
        """
        return self.ur_state_object.get_state_as_string(observation)
