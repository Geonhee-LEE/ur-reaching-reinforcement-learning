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
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# MoveIt
#import moveit_commander
#import moveit_msgs.msg
#import geometry_msgs.msg

# UR5 Utils
from ur_reaching.env.ur_setups import setups
from ur_reaching.env import ur_utils
from controllers_connection import ControllersConnection

class URState(object):

    def __init__(self, max_height, min_height, observations, joint_limits, episode_done_criteria):
        rospy.logdebug("Starting URState Class object...")
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._joint_increment_value = joint_increment_value
        self._observations = observations

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

        # We init the observations
        self.base_orientation = Quaternion()
    	self.target_point = Point()
        self.joints_state = JointState()

        # Arm/Control parameters
        self._ik_params = setups['UR5_6dof']['ik_params']

        rospy.Subscriber("/joint_states", JointState, self.joints_state_callback)
        rospy.Subscriber("/target_blocks_pose", Point, self.target_point_cb)

        # For joint state callback error
        self._ctrl_conn = ControllersConnection(namespace="")

	# Moving object point on the conveyer
    def target_point_cb(self, msg):
    	self.target_point = msg

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.logdebug("Current joint_states READY")
            except Exception as e:
                rospy.logdebug("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.logdebug("ALL SYSTEMS READY")

    def get_xyz(self, q):
        """Get x,y,z coordinates 
        Args:
            q: a numpy array of joints angle positions.
        Returns:
            xyz are the x,y,z coordinates of an end-effector in a Cartesian space.
        """
        mat = ur_utils.forward(q, self._ik_params)
        xyz = mat[:3, 3]
        return xyz

    def get_orientation(self, q):
        """Get Euler angles 
        Args:
            q: a numpy array of joints angle positions.
        Returns:
            xyz are the x,y,z coordinates of an end-effector in a Cartesian space.
        """
        mat = ur_utils.forward(q, self._ik_params)
        orientation = mat[0:3, 0:3]
        roll = -orientation[1, 2]
        pitch = orientation[0, 2]
        yaw = -orientation[0, 1]
        
        return Vector3(roll, pitch, yaw)

    def cvt_quat_to_euler(self, quat):
        euler_rpy = Vector3()
        euler = tf.transformations.euler_from_quaternion(
            [self.quat.x, self.quat.y, self.quat.z, self.quat.w])

        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]
        return euler_rpy

    def get_base_angular_velocity(self):
        return self.base_angular_velocity

    def init_joints_pose(self, init_pos):
        """
        We initialise the Position variable that saves the desired position where we want our
        joints to be
        :param init_pos:
        :return:
        """
        self.current_joint_pose =[]
        self.current_joint_pose = copy.deepcopy(init_pos)
        return self.current_joint_pose

    def get_euclidean_dist(self, p_in, p_pout):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((p_in.x, p_in.y, p_in.z))
        b = numpy.array((p_pout.x, p_pout.y, p_pout.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def get_joint_states(self):
        return self.joints_state

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

    def get_observations(self):
        """
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array
        :return: observation
        """
        joint_states = self.get_joint_states()
        shp_joint_ang = joint_states.position[0]
        shl_joint_ang = joint_states.position[1]
        elb_joint_ang = joint_states.position[2]
        wr1_joint_ang = joint_states.position[3]
        wr2_joint_ang = joint_states.position[4]
        wr3_joint_ang = joint_states.position[5]

        shp_joint_vel = joint_states.velocity[0]
        shl_joint_vel = joint_states.velocity[1]
        elb_joint_vel = joint_states.velocity[2]
        wr1_joint_vel = joint_states.velocity[3]
        wr2_joint_vel = joint_states.velocity[4]
        wr3_joint_vel = joint_states.velocity[5]

        q = [shp_joint_ang, shl_joint_ang, elb_joint_ang, wr1_joint_ang, wr2_joint_ang, wr3_joint_ang]
        eef_x, eef_y, eef_z = get_xyz(q)

        observation = []
        rospy.logdebug("List of Observations==>"+str(self.observations))
        for obs_name in self.observations:
            if obs_name == "shp_joint_ang":
                observation.append(shp_joint_ang)
            elif obs_name == "shl_joint_ang":
                observation.append(shl_joint_ang)
            elif obs_name == "elb_joint_ang":
                observation.append(elb_joint_ang)
            elif obs_name == "wr1_joint_ang":
                observation.append(wr1_joint_ang)
            elif obs_name == "wr2_joint_ang":
                observation.append(wr2_joint_ang)
            elif obs_name == "wr3_joint_ang":
                observation.append(wr3_joint_ang)
            elif obs_name == "shp_joint_vel":
                observation.append(shp_joint_vel)
            elif obs_name == "shl_joint_vel":
                observation.append(shl_joint_vel)
            elif obs_name == "elb_joint_vel":
                observation.append(elb_joint_vel)
            elif obs_name == "wr1_joint_vel":
                observation.append(wr1_joint_vel)
            elif obs_name == "wr2_joint_vel":
                observation.append(wr2_joint_vel)
            elif obs_name == "wr3_joint_vel":
                observation.append(wr3_joint_vel)
            elif obs_name == "eef_x":
                observation.append(eef_x)
            elif obs_name == "eef_y":
                observation.append(eef_y)
            elif obs_name == "eef_z":
                observation.append(eef_z)
            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

        return observation

    def get_action(self, action):
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

if __name__ == "__main__":
    rospy.init_node('ur_state_node', anonymous=True, log_level=rospy.DEBUG)
    max_height = 3.0
    min_height = 0.5
    joint_increment_value = 0.32
    observations = ["shp_joint_ang",
                            "shl_joint_ang",
                            "elb_joint_ang",
                            "wr1_joint_ang",
                            "wr2_joint_ang",
                            "wr3_joint_ang",
                            "shp_joint_vel",
                            "shl_joint_vel",
                            "elb_joint_vel",
                            "wr1_joint_vel",
                            "wr2_joint_vel",
                            "wr3_joint_vel",
                            "eef_x",
                            "eef_y",
                            "eef_z",
                            "orientation_x",
                            "orientation_y",
                            "orientation_z"]

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
    
    joint_limits = {         "shp_max": shp_max,
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

    ur_state = URState( max_height=max_height,
                        min_height=min_height,
                        joint_increment_value=joint_increment_value,
                        observations=observations,
                        joint_limits=joint_limits,
                        episode_done_criteria=episode_done_criteria
                        )

