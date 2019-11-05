#!/usr/bin/env python

import sys
import copy
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import (
    PointStamped,
    Point,
    Quaternion,
    PoseStamped,
    Pose,
)
import moveit_commander
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
import moveit_msgs.msg

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class UR_Moveit_API:

    def __init__(self, boundaries=False):
        self.scene = PlanningSceneInterface("/base_link")
        self.robot = RobotCommander()
        self.group_commander = MoveGroupCommander("manipulator")
        #self.gripper = MoveGroupCommander("gripper")
        self.group_commander.set_end_effector_link('ee_link')
        self.get_basic_infomation()

        if boundaries is True:
            self.add_bounds()

        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states", JointState, self.joint_callback)
        self.joint_pubs = [rospy.Publisher(
            '/%s/command' % name, Float64, queue_size=1) for name in CONTROLLER_NAMES]
        self.gripper_pub = rospy.Publisher(
            '/gripper_prismatic_joint/command', Float64, queue_size=1)
        
        # create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
        self.reset_subscriber = rospy.Subscriber("/ur/reset", String, self.reset)
        rospy.sleep(2)

    def get_basic_infomation(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group_commander.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame, "============")

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group_commander.get_end_effector_link()
        print ("============ End effector: %s" % eef_link, "============")

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", group_names, "============")

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print ("============ Printing robot state")
        print (self.robot.get_current_state(), "============")
        print ("================================================")

    def joint_callback(self, joint_state):
        self.joint_state = joint_state

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group_commander

        waypoints = []
        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
        
    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    '''
    def open_gripper(self, drop=False):
        plan = self.gripper.plan(GRIPPER_DROP if drop else GRIPPER_OPEN)
        return self.gripper.execute(plan, wait=True)
    '''

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

    '''
    def close_gripper(self):
        plan = self.gripper.plan(GRIPPER_CLOSED)
        return self.gripper.execute(plan, wait=True)
    '''
    
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

    def orient_to_target(self, x=None, y=None, angle=None):
        current = self.get_joint_values()
        if x or y:
            angle = np.arctan2(y, x)
        if angle >= 3.1:
            angle = 3.1
        elif angle <= -3.1:
            angle = -3.1
        current[0] = angle
        plan = self.group_commander.plan(current)
        return self.group_commander.execute(plan, wait=True)

    def wrist_rotate(self, angle):
        rotated_values = self.group_commander.get_current_joint_values()
        rotated_values[4] = angle - rotated_values[0]
        if rotated_values[4] > np.pi / 2:
            rotated_values[4] -= np.pi
        elif rotated_values[4] < -(np.pi / 2):
            rotated_values[4] += np.pi
        plan = self.group_commander.plan(rotated_values)
        return self.group_commander.execute(plan, wait=True)

    def get_joint_values(self):
        return self.group_commander.get_current_joint_values()

    def get_current_pose(self):
        return self.group_commander.get_current_pose()

    def move_to_neutral(self):
        print('Moving to neutral...')                  
        plan = self.group_commander.plan(NEUTRAL_VALUES)
        return self.group_commander.execute(plan, wait=True)

    def reset(self, data):
        print ("data: ", data)
        self.move_to_neutral()
        rospy.sleep(0.5)

    def orient_to_pregrasp(self, x, y):
        angle = np.arctan2(y, x)
        return self.move_to_drop(angle)

    def move_to_grasp(self, x, y, z, angle, compensate_control_noise=True):
        if compensate_control_noise:
            x = (x - CONTROL_NOISE_COEFFICIENT_BETA) / CONTROL_NOISE_COEFFICIENT_ALPHA
            y = (y - CONTROL_NOISE_COEFFICIENT_BETA) / CONTROL_NOISE_COEFFICIENT_ALPHA
        
        current_p = self.group_commander.get_current_pose().pose
        p1 = Pose(position=Point(x=x, y=y, z=z), orientation=DOWN_ORIENTATION)
        plan, f = self.group_commander.compute_cartesian_path(
            [current_p, p1], 0.001, 0.0)

        joint_goal = list(plan.joint_trajectory.points[-1].positions)

        first_servo = joint_goal[0]

        joint_goal[4] = (angle - first_servo) % np.pi
        if joint_goal[4] > np.pi / 2:
            joint_goal[4] -= np.pi
        elif joint_goal[4] < -(np.pi / 2):
            joint_goal[4] += np.pi

        try:
            plan = self.group_commander.plan(joint_goal)
        except MoveItCommanderException as e:
            print('Exception while planning')
            traceback.print_exc(e)
            return False

        return self.group_commander.execute(plan, wait=True)

    def move_to_vertical(self, z, force_orientation=True, shift_factor=1.0):
        current_p = self.group_commander.get_current_pose().pose
        current_angle = self.get_joint_values()[4]
        orientation = current_p.orientation if force_orientation else None
        p1 = Pose(position=Point(x=current_p.position.x * shift_factor,
                                 y=current_p.position.y * shift_factor, z=z), orientation=orientation)
        waypoints = [current_p, p1]
        plan, f = self.group_commander.compute_cartesian_path(waypoints, 0.001, 0.0)

        if not force_orientation:
            return self.group_commander.execute(plan, wait=True)
        else:
            if len(plan.joint_trajectory.points) > 0:
                joint_goal = list(plan.joint_trajectory.points[-1].positions)
            else:
                return False

            joint_goal[4] = current_angle

            plan = self.group_commander.plan(joint_goal)
            return self.group_commander.execute(plan, wait=True)

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
        link_names = ['ee_link']
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

    def scenario_plan(self):
        plan = self.group_commander.plan(PREDISCARD_VALUES)
        self.group_commander.execute(plan, wait=True)
        plan = self.group_commander.plan(DISCARD_VALUES)
        self.group_commander.execute(plan, wait=True)
        #self.open_gripper(drop=True)
        plan = self.group_commander.plan(PREDISCARD_VALUES)
        self.group_commander.execute(plan, wait=True)
        self.move_to_neutral()

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group_commander

        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

class StateValidity():
    def __init__(self):
        # prepare msg to interface with moveit
        self.rs = RobotState()
        self.rs.joint_state.name =  JOINT_NAMES
        self.rs.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_states_received = False
        # Publish collision status
        self.collision_pub = rospy.Publisher('/collision_status', Bool, queue_size=1)

        # subscribe to joint joint states
        self.joint_state_subscriber =rospy.Subscriber("/joint_states", JointState, self.jointStatesCB, queue_size=1)
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        
        # Collision status
        self.collision = std_msgs.msg.Bool()
        rospy.loginfo('service is avaiable')

    def jointStatesCB(self, msg):
        '''
        update robot state
        '''
        # Not gripper, but manipulator ur
        if len(msg.position) == 6:
            self.rs.joint_state.position = msg.position #[msg.position[3], msg.position[2], msg.position[0], msg.position[4], msg.position[5], msg.position[6]]
            self.joint_states_received = True
            self.start_collision_checker()

    def start_collision_checker(self):
        while not self.joint_states_received:
            rospy.sleep(0.1)
        self.checkCollision()

    def checkCollision(self):
        '''
        check if robotis in collision
        '''
        if self.getStateValidity().valid:
            rospy.loginfo('robot not in collision, all ok!')
            self.collision = False
        else:
            rospy.logwarn('robot in collision')
            self.collision = True

        self.publishCollisionStatus()
        
    def publishCollisionStatus(self):
        self.collision_pub.publish(self.collision)

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
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("UR_Moveit_API")
    ur_moveit_api = UR_Moveit_API(boundaries=False)
    #ur_moveit_api.move_to_neutral()
    
    svd_node = StateValidity()
    svd_node.start_collision_checker()

    """
    print ("============ Press `Enter` to plan and display a Cartesian path ...")
    raw_input()
    cartesian_plan, fraction = ur_moveit_api.plan_cartesian_path()

    print ("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    raw_input()
    ur_moveit_api.display_trajectory(cartesian_plan)

    print ("============ Press `Enter` to execute a saved path ...")
    raw_input()
    ur_moveit_api.execute_plan(cartesian_plan)
    """


    rospy.spin()


if __name__ == '__main__':
    main()
