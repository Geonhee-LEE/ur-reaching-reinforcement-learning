#!/usr/bin/env python
# Python
import copy
import numpy
import math
import sys

# ROS 
import rospy
import tf

# ROS msg
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# MoveIt
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from move_group_python_interface import MoveGroupPythonInteface

class URState(object):

    def __init__(self, max_height, min_height, abs_max_roll, abs_max_pitch, list_of_observations, joint_limits, episode_done_criteria, joint_increment_value = 0.05, done_reward = -1000.0, alive_reward=10.0, desired_force=7.08, desired_yaw=0.0, weight_r1=1.0, weight_r2=1.0, weight_r3=1.0, weight_r4=1.0, weight_r5=1.0, discrete_division=10, maximum_base_linear_acceleration=3000.0, maximum_base_angular_velocity=20.0, maximum_joint_effort=10.0):
        rospy.logdebug("Starting URState Class object...")
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._joint_increment_value = joint_increment_value
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_force = desired_force
        self._desired_yaw = desired_yaw

        self._weight_r1 = weight_r1
        self._weight_r2 = weight_r2
        self._weight_r3 = weight_r3
        self._weight_r4 = weight_r4
        self._weight_r5 = weight_r5

        self._list_of_observations = list_of_observations

        # Dictionary with the max and min of each of the joints
        self._joint_limits = joint_limits

        # Maximum base linear acceleration values
        self.maximum_base_linear_acceleration = maximum_base_linear_acceleration

        # Maximum Angular Velocity value
        # By maximum means the value that we consider relevant maximum, the sensor might pick up higher
        # But its an equilibrium between precission and number of divisions of the sensors data.
        self.maximum_base_angular_velocity = maximum_base_angular_velocity

        self.maximum_joint_effort = maximum_joint_effort

        # List of all the Done Episode Criteria
        self._episode_done_criteria = episode_done_criteria
        assert len(self._episode_done_criteria) != 0, "Episode_done_criteria list is empty. Minimum one value"

        self._discrete_division = discrete_division

        # We init the observation ranges and We create the bins now for all the observations
        self.init_bins()

        self.base_position = Point()
        self.base_orientation = Quaternion()
        self.base_angular_velocity = Vector3()
        self.base_linear_acceleration = Vector3()
        self.contact_force = Vector3()
        self.joints_state = JointState()

        # Odom we only use it for the height detection and planar position ,
        #  because in real robots this data is not trivial.
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber("/ur/imu/data", Imu, self.imu_callback)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        rospy.Subscriber("/lowerleg_contactsensor_state", ContactsState, self.contact_callback)
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/joint_states", JointState, self.joints_state_callback)

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        '''
        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                data_pose = rospy.wait_for_message("/odom", Odometry, timeout=0.1)
                self.base_position = data_pose.pose.pose.position
                rospy.logdebug("Current odom READY")
            except:
                rospy.logdebug("Current odom pose not ready yet, retrying for getting robot base_position")

        imu_data = None
        while imu_data is None and not rospy.is_shutdown():
            try:
                imu_data = rospy.wait_for_message("/ur/imu/data", Imu, timeout=0.1)
                self.base_orientation = imu_data.orientation
                self.base_angular_velocity = imu_data.angular_velocity
                self.base_linear_acceleration = imu_data.linear_acceleration
                rospy.logdebug("Current imu_data READY")
            except:
                rospy.logdebug("Current imu_data not ready yet, retrying for getting robot base_orientation, and base_linear_acceleration")

        contacts_data = None
        while contacts_data is None and not rospy.is_shutdown():
            try:
                contacts_data = rospy.wait_for_message("/lowerleg_contactsensor_state", ContactsState, timeout=0.1)
                for state in contacts_data.states:
                    self.contact_force = state.total_wrench.force
                rospy.logdebug("Current contacts_data READY")
            except:
                rospy.logdebug("Current contacts_data not ready yet, retrying")
        '''
        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.logdebug("Current joint_states READY")
            except Exception as e:
                rospy.logdebug("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.logdebug("ALL SYSTEMS READY")

    def set_desired_world_point(self, x, y, z):
        """
        Point where you want the UR to be
        :return:
        """
        self.desired_world_point.x = x
        self.desired_world_point.y = y
        self.desired_world_point.z = z

    def get_base_height(self):
        height = self.base_position.z
        rospy.logdebug("BASE-HEIGHT="+str(height))
        return height

    def get_base_rpy(self):
        euler_rpy = Vector3()
        euler = tf.transformations.euler_from_quaternion(
            [self.base_orientation.x, self.base_orientation.y, self.base_orientation.z, self.base_orientation.w])

        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]
        return euler_rpy

    def get_base_angular_velocity(self):
        return self.base_angular_velocity

    def get_base_linear_acceleration(self):
        return self.base_linear_acceleration

    def get_distance_from_point(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((self.base_position.x, self.base_position.y, self.base_position.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def get_contact_force_magnitude(self):
        """
        You will see that because the X axis is the one pointing downwards, it will be the one with
        higher value when touching the floor
        For a Robot of total mas of 0.55Kg, a gravity of 9.81 m/sec**2, Weight = 0.55*9.81=5.39 N
        Falling from around 5centimetres ( negligible height ), we register peaks around
        Fx = 7.08 N
        :return:
        """
        contact_force = self.contact_force
        contact_force_np = numpy.array((contact_force.x, contact_force.y, contact_force.z))
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude

    def get_joint_states(self):
        return self.joints_state

    def odom_callback(self,msg):
        self.base_position = msg.pose.pose.position

    def imu_callback(self,msg):
        self.base_orientation = msg.orientation
        self.base_angular_velocity = msg.angular_velocity
        self.base_linear_acceleration = msg.linear_acceleration

    def contact_callback(self,msg):
        """
        /lowerleg_contactsensor_state/states[0]/contact_positions ==> PointContact in World
        /lowerleg_contactsensor_state/states[0]/contact_normals ==> NormalContact in World

        ==> One is an array of all the forces, the other total,
         and are relative to the contact link referred to in the sensor.
        /lowerleg_contactsensor_state/states[0]/wrenches[]
        /lowerleg_contactsensor_state/states[0]/total_wrench
        :param msg:
        :return:
        """
        for state in msg.states:
            self.contact_force = state.total_wrench.force

    def joints_state_callback(self,msg):
        self.joints_state = msg

    def ur_height_ok(self):

        height_ok = self._min_height <= self.get_base_height() < self._max_height
        return height_ok

    def ur_orientation_ok(self):

        orientation_rpy = self.get_base_rpy()
        roll_ok = self._abs_max_roll > abs(orientation_rpy.x)
        pitch_ok = self._abs_max_pitch > abs(orientation_rpy.y)
        orientation_ok = roll_ok and pitch_ok
        return orientation_ok

    def calculate_reward_joint_position(self, weight=1.0):
        """
        We calculate reward base on the joints configuration. The more near 0 the better.
        :return:
        """
        acumulated_joint_pos = 0.0
        for joint_pos in self.joints_state.position:
            # Abs to remove sign influence, it doesnt matter the direction of turn.
            acumulated_joint_pos += abs(joint_pos)
            rospy.logdebug("calculate_reward_joint_position>>acumulated_joint_pos=" + str(acumulated_joint_pos))
        reward = weight * acumulated_joint_pos
        rospy.logdebug("calculate_reward_joint_position>>reward=" + str(reward))
        return reward

    def calculate_reward_joint_effort(self, weight=1.0):
        """
        We calculate reward base on the joints effort readings. The more near 0 the better.
        :return:
        """
        acumulated_joint_effort = 0.0
        for joint_effort in self.joints_state.effort:
            # Abs to remove sign influence, it doesnt matter the direction of the effort.
            acumulated_joint_effort += abs(joint_effort)
            rospy.logdebug("calculate_reward_joint_effort>>joint_effort=" + str(joint_effort))
            rospy.logdebug("calculate_reward_joint_effort>>acumulated_joint_effort=" + str(acumulated_joint_effort))
        reward = weight * acumulated_joint_effort
        rospy.logdebug("calculate_reward_joint_effort>>reward=" + str(reward))
        return reward

    def calculate_reward_contact_force(self, weight=1.0):
        """
        We calculate reward base on the contact force.
        The nearest to the desired contact force the better.
        We use exponential to magnify big departures from the desired force.
        Default ( 7.08 N ) desired force was taken from reading of the robot touching
        the ground from a negligible height of 5cm.
        :return:
        """
        force_magnitude = self.get_contact_force_magnitude()
        force_displacement = force_magnitude - self._desired_force

        rospy.logdebug("calculate_reward_contact_force>>force_magnitude=" + str(force_magnitude))
        rospy.logdebug("calculate_reward_contact_force>>force_displacement=" + str(force_displacement))
        # Abs to remove sign
        reward = weight * abs(force_displacement)
        rospy.logdebug("calculate_reward_contact_force>>reward=" + str(reward))
        return reward

    def calculate_reward_orientation(self, weight=1.0):
        """
        We calculate the reward based on the orientation.
        The more its closser to 0 the better because it means its upright
        desired_yaw is the yaw that we want it to be.
        to praise it to have a certain orientation, here is where to set it.
        :return:
        """
        curren_orientation = self.get_base_rpy()
        yaw_displacement = curren_orientation.z - self._desired_yaw
        rospy.logdebug("calculate_reward_orientation>>[R,P,Y]=" + str(curren_orientation))
        acumulated_orientation_displacement = abs(curren_orientation.x) + abs(curren_orientation.y) + abs(yaw_displacement)
        reward = weight * acumulated_orientation_displacement
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def calculate_reward_distance_from_des_point(self, weight=1.0):
        """
        We calculate the distance from the desired point.
        The closser the better
        :param weight:
        :return:reward
        """
        distance = self.get_distance_from_point(self.desired_world_point)
        reward = weight * distance
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def calculate_total_reward(self):
        """
        We consider VERY BAD REWARD -7 or less
        Perfect reward is 0.0, and total reward 1.0.
        The defaults values are chosen so that when the robot has fallen or very extreme joint config:
        r1 = -8.04
        r2 = -8.84
        r3 = -7.08
        r4 = -10.0 ==> We give priority to this, giving it higher value.
        :return:
        """

        r1 = self.calculate_reward_joint_position(self._weight_r1)
        r2 = self.calculate_reward_joint_effort(self._weight_r2)
        # Desired Force in Newtons, taken form idle contact with 9.81 gravity.
        r3 = self.calculate_reward_contact_force(self._weight_r3)
        r4 = self.calculate_reward_orientation(self._weight_r4)
        r5 = self.calculate_reward_distance_from_des_point(self._weight_r5)

        # The sign depend on its function.
        total_reward = self._alive_reward - r1 - r2 - r3 - r4 - r5

        rospy.logdebug("###############")
        rospy.logdebug("alive_bonus=" + str(self._alive_reward))
        rospy.logdebug("r1 joint_position=" + str(r1))
        rospy.logdebug("r2 joint_effort=" + str(r2))
        rospy.logdebug("r3 contact_force=" + str(r3))
        rospy.logdebug("r4 orientation=" + str(r4))
        rospy.logdebug("r5 distance=" + str(r5))
        rospy.logdebug("total_reward=" + str(total_reward))
        rospy.logdebug("###############")

        return total_reward

    def get_observations(self):
        """
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array of the:
        1) distance from desired point in meters
        2) The pitch orientation in radians
        3) the Roll orientation in radians
        4) the Yaw orientation in radians
        5) Force in contact sensor in Newtons
        6-7-8) State of the 3 joints in radians

        observation = [distance_from_desired_point,
                 base_roll,
                 base_pitch,
                 base_yaw,
                 base_angular_vel_x,
                 base_angular_vel_y,
                 base_angular_vel_z,
                 base_linear_acceleration_x,
                 base_linear_acceleration_y,
                 base_linear_acceleration_z,
                 contact_force,
                 joint_states_haa,
                 joint_states_hfe,
                 joint_states_kfe]

        :return: observation
        """

        distance_from_desired_point = self.get_distance_from_point(self.desired_world_point)

        base_orientation = self.get_base_rpy()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z

        base_angular_velocity = self.get_base_angular_velocity()
        base_angular_vel_x = base_angular_velocity.x
        base_angular_vel_y = base_angular_velocity.y
        base_angular_vel_z = base_angular_velocity.z

        base_linear_acceleration = self.get_base_linear_acceleration()
        base_linear_acceleration_x = base_linear_acceleration.x
        base_linear_acceleration_y = base_linear_acceleration.y
        base_linear_acceleration_z = base_linear_acceleration.z

        contact_force = self.get_contact_force_magnitude()

        joint_states = self.get_joint_states()
        joint_states_haa = joint_states.position[0]
        joint_states_hfe = joint_states.position[1]
        joint_states_kfe = joint_states.position[2]

        joint_effort_haa = joint_states.effort[0]
        joint_effort_hfe = joint_states.effort[1]
        joint_effort_kfe = joint_states.effort[2]

        observation = []
        rospy.logdebug("List of Observations==>"+str(self._list_of_observations))
        for obs_name in self._list_of_observations:
            if obs_name == "distance_from_desired_point":
                observation.append(distance_from_desired_point)
            elif obs_name == "base_roll":
                observation.append(base_roll)
            elif obs_name == "base_pitch":
                observation.append(base_pitch)
            elif obs_name == "base_yaw":
                observation.append(base_yaw)
            elif obs_name == "contact_force":
                observation.append(contact_force)
            elif obs_name == "joint_states_haa":
                observation.append(joint_states_haa)
            elif obs_name == "joint_states_hfe":
                observation.append(joint_states_hfe)
            elif obs_name == "joint_states_kfe":
                observation.append(joint_states_kfe)
            elif obs_name == "joint_effort_haa":
                observation.append(joint_effort_haa)
            elif obs_name == "joint_effort_hfe":
                observation.append(joint_effort_hfe)
            elif obs_name == "joint_effort_kfe":
                observation.append(joint_effort_kfe)
            elif obs_name == "base_angular_vel_x":
                observation.append(base_angular_vel_x)
            elif obs_name == "base_angular_vel_y":
                observation.append(base_angular_vel_y)
            elif obs_name == "base_angular_vel_z":
                observation.append(base_angular_vel_z)
            elif obs_name == "base_linear_acceleration_x":
                observation.append(base_linear_acceleration_x)
            elif obs_name == "base_linear_acceleration_y":
                observation.append(base_linear_acceleration_y)
            elif obs_name == "base_linear_acceleration_z":
                observation.append(base_linear_acceleration_z)
            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

        return observation

    def get_state_as_string(self, observation):
        """
        This function will do two things:
        1) It will make discrete the observations
        2) Will convert the discrete observations in to state tags strings
        :param observation:
        :return: state
        """
        observations_discrete = self.assign_bins(observation)
        string_state = ''.join(map(str, observations_discrete))
        rospy.logdebug("STATE==>"+str(string_state))
        return string_state

    def assign_bins(self, observation):
        """
        Will make observations discrete by placing each value into its corresponding bin
        :param observation:
        :return:
        """
        rospy.logdebug("Observations>>"+str(observation))

        state_discrete = numpy.zeros(len(self._list_of_observations), dtype=numpy.int32)
        for i in range(len(self._list_of_observations)):
            # We convert to int because anyway it will be round floats. We add Right True to include limits
            # Ex: [-20, 0, 20], value=-20 ==> index=0, In right = False, would be index=1
            state_discrete[i] = int(numpy.digitize(observation[i], self._bins[i], right=True))
            #rospy.logdebug("bin="+str(self._bins[i])+"obs="+str(observation[i])+",end_val="+str(state_discrete[i]))

        rospy.logdebug(str(state_discrete))
        return state_discrete

    def init_bins(self):
        """
        We initalise all related to the bins
        :return:
        """
        self.fill_observations_ranges()
        self.create_bins()

    def fill_observations_ranges(self):
        """
        We create the dictionary for the ranges of the data related to each observation
        :return:
        """
        self._obs_range_dict = {}
        for obs_name in self._list_of_observations:

            if obs_name == "distance_from_desired_point":
                # We consider the range as based on the range of distance allowed in height
                delta = self._max_height - self._min_height
                max_value = delta
                min_value = -delta
            elif obs_name == "base_roll":
                max_value = self._abs_max_roll
                min_value = -self._abs_max_roll
            elif obs_name == "base_pitch":
                max_value = self._abs_max_pitch
                min_value = -self._abs_max_pitch
            elif obs_name == "base_yaw":
                # We consider that 360 degrees is max range
                max_value = 2*math.pi
                min_value = -2*math.pi
            elif obs_name == "contact_force":
                # We consider that no force is the minimum, and the maximum is 2 times the desired
                # We dont want to make a very big range because we might loose the desired force
                # in the middle.
                max_value = 2*self._desired_force
                min_value = 0.0

            elif obs_name == "joint_states_haa":
                # We consider the URDF maximum values
                max_value = self._joint_limits["haa_max"]
                min_value = self._joint_limits["haa_min"]
            elif obs_name == "joint_states_hfe":
                max_value = self._joint_limits["hfe_max"]
                min_value = self._joint_limits["hfe_min"]
            elif obs_name == "joint_states_kfe":
                max_value = self._joint_limits["kfe_max"]
                min_value = self._joint_limits["kfe_min"]

            elif obs_name == "joint_effort_haa":
                # We consider the URDF maximum values
                max_value = self.maximum_joint_effort
                min_value = -self.maximum_joint_effort
            elif obs_name == "joint_effort_hfe":
                max_value = self.maximum_joint_effort
                min_value = -self.maximum_joint_effort
            elif obs_name == "joint_effort_kfe":
                max_value = self.maximum_joint_effort
                min_value = -self.maximum_joint_effort


            elif obs_name == "base_angular_vel_x":
                max_value = self.maximum_base_angular_velocity
                min_value = -self.maximum_base_angular_velocity
            elif obs_name == "base_angular_vel_y":
                max_value = self.maximum_base_angular_velocity
                min_value = -self.maximum_base_angular_velocity
            elif obs_name == "base_angular_vel_z":
                max_value = self.maximum_base_angular_velocity
                min_value = -self.maximum_base_angular_velocity

            elif obs_name == "base_linear_acceleration_x":
                max_value = self.maximum_base_linear_acceleration
                min_value = -self.maximum_base_linear_acceleration
            elif obs_name == "base_linear_acceleration_y":
                max_value = self.maximum_base_linear_acceleration
                min_value = -self.maximum_base_linear_acceleration
            elif obs_name == "base_linear_acceleration_z":
                max_value = self.maximum_base_linear_acceleration
                min_value = -self.maximum_base_linear_acceleration

            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

            self._obs_range_dict[obs_name] = [min_value,max_value]

    def create_bins(self):
        """
        We create the Bins for the discretization of the observations
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._joint_increment_value = joint_increment_value
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_force = desired_force
        self._desired_yaw = desired_yaw


        :return:bins
        """

        number_of_observations = len(self._list_of_observations)
        parts_we_disrcetize = self._discrete_division
        rospy.logdebug("Parts to discretise==>"+str(parts_we_disrcetize))
        self._bins = numpy.zeros((number_of_observations, parts_we_disrcetize))
        for counter in range(number_of_observations):
            obs_name = self._list_of_observations[counter]
            min_value = self._obs_range_dict[obs_name][0]
            max_value = self._obs_range_dict[obs_name][1]
            self._bins[counter] = numpy.linspace(min_value, max_value, parts_we_disrcetize)

            rospy.logdebug("bins==>" + str(self._bins[counter]))

    def init_joints_pose(self, des_init_pos):
        """
        We initialise the Position variable that saves the desired position where we want our
        joints to be
        :param init_pos:
        :return:
        """
        self.current_joint_pose =[]
        self.current_joint_pose = copy.deepcopy(des_init_pos)
        return self.current_joint_pose

    def get_action_to_position(self, action):
        """
        Here we have the ACtions number to real joint movement correspondance.
        :param action: Integer that goes from 0 to 6, because we have 7 actions.
        :return:
        """

        rospy.logdebug("current joint pose>>>"+str(self.current_joint_pose))
        rospy.logdebug("Action Number>>>"+str(action))
        # We dont want to jump unless the action jump is selected

        if action == 0: #Increment 
            rospy.logdebug("Action Decided:Increment shp_joint>>>")
            self.current_joint_pose[0] += self._joint_increment_value
        elif action == 1: #Increment 
            rospy.logdebug("Action Decided:Increment shl_joint>>>")
            self.current_joint_pose[1] -= self._joint_increment_value
        elif action == 2: #Increment 
            rospy.logdebug("Action Decided:Increment elb_joint>>>")
            self.current_joint_pose[2] += self._joint_increment_value
        elif action == 3: #Increment 
            rospy.logdebug("Action Decided:Increment wr1_joint>>>")
            self.current_joint_pose[3] -= self._joint_increment_value
        elif action == 4:  # Increment
            rospy.logdebug("Action Decided:Increment wr2_joint>>>")
            self.current_joint_pose[4] += self._joint_increment_value
        elif action == 5:  # Increment
            rospy.logdebug("Action Decided:Increment wr3_joint>>>")
            # We get the Value Used for the Knee charged position
            self.current_joint_pose[5] += self._joint_increment_value

        rospy.logdebug("action to move joint states>>>" + str(self.current_joint_pose))

        self.clamp_to_joint_limits()

        return self.current_joint_pose

    def clamp_to_joint_limits(self):
        """
        clamps self.current_joint_pose based on the joint limits
        self._joint_limits
        {
         "shp_max": shp_max,
         "shp_min": shp_min,
         ...
         }
        :return:
        """

        rospy.logdebug("Clamping current_joint_pose>>>" + str(self.current_joint_pose))
        shp_joint_value = self.current_joint_pose[0]
        shl_joint_value = self.current_joint_pose[1]
        elb_joint_value = self.current_joint_pose[2]
        wr1_joint_value = self.current_joint_pose[3]
        wr2_joint_value = self.current_joint_pose[4]
        wr3_joint_value = self.current_joint_pose[5]

        self.current_joint_pose[0] = max(min(shp_joint_value, self._joint_limits["shp_max"]),
                                         self._joint_limits["shp_min"])
        self.current_joint_pose[1] = max(min(shl_joint_value, self._joint_limits["shl_max"]),
                                         self._joint_limits["shl_min"])
        self.current_joint_pose[2] = max(min(elb_joint_value, self._joint_limits["elb_max"]),
                                         self._joint_limits["elb_min"])
        self.current_joint_pose[3] = max(min(wr1_joint_value, self._joint_limits["wr1_max"]),
                                         self._joint_limits["wr1_min"])
        self.current_joint_pose[4] = max(min(wr2_joint_value, self._joint_limits["wr2_max"]),
                                         self._joint_limits["wr2_min"])
        self.current_joint_pose[5] = max(min(wr3_joint_value, self._joint_limits["wr3_max"]),
                                         self._joint_limits["wr3_min"])

        rospy.logdebug("DONE Clamping current_joint_pose>>>" + str(self.current_joint_pose))

    def process_data(self):
        """
        We return the total reward based on the state in which we are in and if its done or not
        ( it fell basically )
        :return: reward, done
        """

        if "ur_minimum_height" in self._episode_done_criteria:
            ur_height_ok = self.ur_height_ok()
        else:
            rospy.logdebug("ur_height_ok NOT TAKEN INTO ACCOUNT")
            ur_height_ok = True

        if "ur_vertical_orientation" in self._episode_done_criteria:
            ur_orientation_ok = self.ur_orientation_ok()
        else:
            rospy.logdebug("ur_orientation_ok NOT TAKEN INTO ACCOUNT")
            ur_orientation_ok = True

        rospy.logdebug("ur_height_ok="+str(ur_height_ok))
        rospy.logdebug("ur_orientation_ok=" + str(ur_orientation_ok))

        done = not(ur_height_ok and ur_orientation_ok)
        if done:
            rospy.logerr("It fell, so the reward has to be very low")
            total_reward = self._done_reward
        else:
            rospy.logdebug("Calculate normal reward because it didn't fall.")
            total_reward = self.calculate_total_reward()

        return total_reward, done

    def testing_loop(self):

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.calculate_total_reward()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('ur_state_node', anonymous=True, log_level=rospy.DEBUG)
    max_height = 3.0
    min_height = 0.5
    max_incl = 1.57
    joint_increment_value = 0.32
    list_of_observations = ["base_roll",
                            "base_pitch",
                            "base_angular_vel_x",
                            "base_angular_vel_y",
                            "base_angular_vel_z",
                            "base_linear_acceleration_x",
                            "base_linear_acceleration_y",
                            "base_linear_acceleration_z"]

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
    
    joint_limits = {"shp_max": shp_max,
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
						     "wr3_min": wr3_min}
    episode_done_criteria = [ "minimum_height",
                              "vertical_orientation"]
    done_reward = -1000.0
    alive_reward = 100.0
    desired_force = 7.08
    desired_yaw = 0.0
    weight_r1 = 0.0 # Weight for joint positions ( joints in the zero is perfect )
    weight_r2 = 0.0 # Weight for joint efforts ( no efforts is perfect )
    weight_r3 = 0.0 # Weight for contact force similar to desired ( weight of ur )
    weight_r4 = 10.0 # Weight for orientation ( vertical is perfect )
    weight_r5 = 10.0 # Weight for distance from desired point ( on the point is perfect )
    discrete_division = 10
    maximum_base_linear_acceleration = 3000.0
    ur_state = URState(   max_height=max_height,
                                    min_height=min_height,
                                    abs_max_roll=max_incl,
                                    abs_max_pitch=max_incl,
                                    joint_increment_value=joint_increment_value,
                                    list_of_observations=list_of_observations,
                                    joint_limits=joint_limits,
                                    episode_done_criteria=episode_done_criteria,
                                    done_reward=done_reward,
                                    alive_reward=alive_reward,
                                    desired_force=desired_force,
                                    desired_yaw=desired_yaw,
                                    weight_r1=weight_r1,
                                    weight_r2=weight_r2,
                                    weight_r3=weight_r3,
                                    weight_r4=weight_r4,
                                    weight_r5=weight_r5,
                                    discrete_division=discrete_division,
                                    maximum_base_linear_acceleration=maximum_base_linear_acceleration
                                                )
    ur_state.testing_loop()
