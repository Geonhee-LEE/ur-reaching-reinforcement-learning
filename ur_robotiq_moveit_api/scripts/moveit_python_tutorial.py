#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import (
    PointStamped,
    Point,
    Quaternion,
    PoseStamped,
    Pose,
)
from moveit_commander import *
from moveit_msgs.srv import *
from moveit_msgs.msg import *

from moveit_commander.exception import MoveItCommanderException

import numpy as np
from config import *

# Python API(https://github.com/mikeferguson/moveit_python)
#from moveit_python import *
from moveit_python import PlanningSceneInterface
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class UR_Moveit_API:

    def __init__(self, boundaries=False):
        self.scene = PlanningSceneInterface("/base_link")
        self.commander = MoveGroupCommander("manipulator")
        self.gripper = MoveGroupCommander("gripper")
        self.commander.set_end_effector_link('eef_link')

        if boundaries is True:
            self.add_bounds()

        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states", JointState, self.joint_callback)
        self.joint_pubs = [rospy.Publisher(
            '/%s/command' % name, Float64, queue_size=1) for name in CONTROLLER_NAMES]
        self.gripper_pub = rospy.Publisher(
            '/gripper_prismatic_joint/command', Float64, queue_size=1)

        self.reset_subscriber = rospy.Subscriber("/ur/reset", String, self.reset)
        rospy.sleep(2)

    def joint_callback(self, joint_state):
        self.joint_state = joint_state

    def open_gripper(self, drop=False):
        plan = self.gripper.plan(GRIPPER_DROP if drop else GRIPPER_OPEN)
        return self.gripper.execute(plan, wait=True)

    def add_bounds(self):
        print("Complete to add_bounds")
        # size_x, size_y, size_z, x, y, z
        self.scene.addBox('table', 1.0, 1.0, 1.0 , .0, .0, -0.5)

        # add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
        #self.scene.addCube("my_cube", 0.1, 1, 0, 0.5)
        
        # Remove the cube
        #self.scene.removeCollisionObject("my_cube")

    def remove_bounds(self):
        for obj in self.scene.get_objects().keys():
            self.scene.remove_world_object(obj)

    def close_gripper(self):
        plan = self.gripper.plan(GRIPPER_CLOSED)
        return self.gripper.execute(plan, wait=True)

    def get_ik_client(self, request):
        rospy.wait_for_service('/compute_ik')
        inverse_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        ret = inverse_ik(request)
        if ret.error_code.val != 1:
            return None
        return ret.solution.joint_state

    def get_fk_client(self, header, link_names, robot_state):
        rospy.wait_for_service('/compute_fk')
        fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        ret = fk(header, link_names, robot_state)
        if ret.error_code.val != 1:
            return None
        return ret.pose_stamped

    def eval_grasp(self, threshold=.0001, manual=False):
        if manual:
            user_input = None
            while user_input not in ('y', 'n', 'r'):
                user_input = raw_input(
                    'Successful grasp? [(y)es/(n)o/(r)edo]: ')
            if user_input == 'y':
                return 1, None
            elif user_input == 'n':
                return 0, None
            else:
                return -1, None
        else:
            current = np.array(self.gripper.get_current_joint_values())
            target = np.array(GRIPPER_CLOSED)
            error = current[0] - target[0]
            return error > threshold, error

    def orient_to_target(self, x=None, y=None, angle=None):
        current = self.get_joint_values()
        if x or y:
            angle = np.arctan2(y, x)
        if angle >= 3.1:
            angle = 3.1
        elif angle <= -3.1:
            angle = -3.1
        current[0] = angle
        plan = self.commander.plan(current)
        return self.commander.execute(plan, wait=True)

    def wrist_rotate(self, angle):
        rotated_values = self.commander.get_current_joint_values()
        rotated_values[4] = angle - rotated_values[0]
        if rotated_values[4] > np.pi / 2:
            rotated_values[4] -= np.pi
        elif rotated_values[4] < -(np.pi / 2):
            rotated_values[4] += np.pi
        plan = self.commander.plan(rotated_values)
        return self.commander.execute(plan, wait=True)

    def get_joint_values(self):
        return self.commander.get_current_joint_values()

    def get_current_pose(self):
        return self.commander.get_current_pose()

    def move_to_neutral(self):
        print('Moving to neutral...')
        NEUTRAL_VALUES = [-1.57, -1.0, 
                  1.5, -2.50,
                  -1.57, 0.0]
                  
        plan = self.commander.plan(NEUTRAL_VALUES)
        return self.commander.execute(plan, wait=True)

    def move_to_drop(self, angle=None):
        drop_positions = DROPPING_VALUES[:]
        if angle:
            drop_positions[0] = angle
        plan = self.commander.plan(drop_positions)
        return self.commander.execute(plan, wait=True)

    def move_to_empty(self):
        plan = self.commander.plan(EMPTY_VALUES)
        return self.commander.execute(plan, wait=True)

    def move_to_reset(self):
        print('Moving to reset...')
        plan = self.commander.plan(RESET_VALUES)
        return self.commander.execute(plan, wait=True)

    def reset(self, data):
        print ("data: ", data)
        self.move_to_reset()
        rospy.sleep(0.5)

    def orient_to_pregrasp(self, x, y):
        angle = np.arctan2(y, x)
        return self.move_to_drop(angle)

    def move_to_grasp(self, x, y, z, angle, compensate_control_noise=True):
        if compensate_control_noise:
            x = (x - CONTROL_NOISE_COEFFICIENT_BETA) / CONTROL_NOISE_COEFFICIENT_ALPHA
            y = (y - CONTROL_NOISE_COEFFICIENT_BETA) / CONTROL_NOISE_COEFFICIENT_ALPHA
        
        current_p = self.commander.get_current_pose().pose
        p1 = Pose(position=Point(x=x, y=y, z=z), orientation=DOWN_ORIENTATION)
        plan, f = self.commander.compute_cartesian_path(
            [current_p, p1], 0.001, 0.0)

        joint_goal = list(plan.joint_trajectory.points[-1].positions)

        first_servo = joint_goal[0]

        joint_goal[4] = (angle - first_servo) % np.pi
        if joint_goal[4] > np.pi / 2:
            joint_goal[4] -= np.pi
        elif joint_goal[4] < -(np.pi / 2):
            joint_goal[4] += np.pi

        try:
            plan = self.commander.plan(joint_goal)
        except MoveItCommanderException as e:
            print('Exception while planning')
            traceback.print_exc(e)
            return False

        return self.commander.execute(plan, wait=True)

    def move_to_vertical(self, z, force_orientation=True, shift_factor=1.0):
        current_p = self.commander.get_current_pose().pose
        current_angle = self.get_joint_values()[4]
        orientation = current_p.orientation if force_orientation else None
        p1 = Pose(position=Point(x=current_p.position.x * shift_factor,
                                 y=current_p.position.y * shift_factor, z=z), orientation=orientation)
        waypoints = [current_p, p1]
        plan, f = self.commander.compute_cartesian_path(waypoints, 0.001, 0.0)

        if not force_orientation:
            return self.commander.execute(plan, wait=True)
        else:
            if len(plan.joint_trajectory.points) > 0:
                joint_goal = list(plan.joint_trajectory.points[-1].positions)
            else:
                return False

            joint_goal[4] = current_angle

            plan = self.commander.plan(joint_goal)
            return self.commander.execute(plan, wait=True)

    def move_to_target(self, target):
        assert len(target) >= 6, 'Invalid target command'
        for i, pos in enumerate(target):
            self.joint_pubs[i].publish(pos)

    def move_to_joint_position(self, joints):
        """
        Adds the given joint values to the current joint values, moves to position
        """
        joint_state = self.joint_state
        joint_dict = dict(zip(joint_state.name, joint_state.position))
        for i in range(len(JOINT_NAMES)):
            joint_dict[JOINT_NAMES[i]] += joints[i]
        joint_state = JointState()
        joint_state.name = JOINT_NAMES
        joint_goal = [joint_dict[joint] for joint in JOINT_NAMES]
        joint_goal = np.clip(np.array(joint_goal), JOINT_MIN, JOINT_MAX)
        joint_state.position = joint_goal
        header = Header()
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        link_names = ['eef_link']
        position = self.get_fk_client(header, link_names, robot_state)
        target_p = position[0].pose.position
        x, y, z = target_p.x, target_p.y, target_p.z
        conditions = [
            x <= BOUNDS_LEFTWALL,
            x >= BOUNDS_RIGHTWALL,
            y <= BOUNDS_BACKWALL,
            y >= BOUNDS_FRONTWALL,
            z <= BOUNDS_FLOOR,
            z >= 0.15
        ]
        print("Target Position: %0.4f, %0.4f, %0.4f" % (x, y, z))
        for condition in conditions:
            if not condition:
                return
        self.move_to_target(joint_goal)
        rospy.sleep(0.15)

    def sweep_arena(self):
        self.remove_bounds()
        self.move_to_drop(.8)
        plan = self.commander.plan(TL_CORNER[0])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(TL_CORNER[1])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(L_SWEEP[0])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(L_SWEEP[1])
        self.commander.execute(plan, wait=True)

        self.move_to_drop(-.8)
        plan = self.commander.plan(BL_CORNER[0])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(BL_CORNER[1])
        self.commander.execute(plan, wait=True)

        self.move_to_drop(-2.45)
        plan = self.commander.plan(BR_CORNER[0])
        self.commander.execute(plan, wait=True)
        plan = self.commander.plan(BR_CORNER[1])
        self.commander.execute(plan, wait=True)

        self.move_to_drop(2.3)
        plan = self.commander.plan(TR_CORNER[0])
        self.commander.execute(plan, wait=True)
        plan = self.commander.plan(TR_CORNER[1])
        self.commander.execute(plan, wait=True)
        self.add_bounds()

    def discard_object(self):
        plan = self.commander.plan(PREDISCARD_VALUES)
        self.commander.execute(plan, wait=True)
        plan = self.commander.plan(DISCARD_VALUES)
        self.commander.execute(plan, wait=True)
        self.open_gripper(drop=True)
        plan = self.commander.plan(PREDISCARD_VALUES)
        self.commander.execute(plan, wait=True)
        self.move_to_neutral()


class StateValidity():
    def __init__(self):
        # prepare msg to interface with moveit
        self.rs = RobotState()
        self.rs.joint_state.name =  JOINT_NAMES
        self.rs.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_states_received = False
        # subscribe to joint joint states
        self.joint_state_subscriber =rospy.Subscriber("/joint_states", JointState, self.jointStatesCB, queue_size=1)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')

    def jointStatesCB(self, msg):
        '''
        update robot state
        '''
        self.rs.joint_state.position = [msg.position[3], msg.position[2], msg.position[0], msg.position[4], msg.position[5], msg.position[6]]
        self.joint_states_received = True

    def start_collision_checker(self):
        while not self.joint_states_received:
            rospy.sleep(0.1)
        rospy.loginfo('joint states received! continue')
        self.checkCollision()
        rospy.spin()

    def checkCollision(self):
        '''
        check if robotis in collision
        '''
        if self.getStateValidity().valid:
            rospy.loginfo('robot not in collision, all ok!')
        else:
            rospy.logwarn('robot in collision')

    def getStateValidity(self, group_name='manipulator', constraints=None):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints

        # State validity Service call
        result = self.sv_srv.call(gsvr)
        return result


def main():
    rospy.init_node("UR_Moveit_API")
    ur_moveit_api = UR_Moveit_API(boundaries=True)
    ur_moveit_api.move_to_neutral()
    
    collision_checker_node = StateValidity()
    collision_checker_node.start_collision_checker()


    rospy.spin()


if __name__ == '__main__':
    main()
