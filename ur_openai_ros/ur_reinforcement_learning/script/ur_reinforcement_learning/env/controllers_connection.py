#!/usr/bin/env python
import sys
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse
from controller_manager_msgs.srv import LoadController, LoadControllerRequest, LoadControllerResponse
from controller_manager_msgs.srv import UnloadController, UnloadControllerRequest, UnloadControllerResponse


class ControllersConnection():
    
    def __init__(self, namespace):

        self.switch_service_name = '/controller_manager/switch_controller'
        self.switch_service = rospy.ServiceProxy(self.switch_service_name, SwitchController)
        
        self.load_service_name = '/controller_manager/load_controller'
        self.load_service = rospy.ServiceProxy(self.load_service_name, LoadController)

        self.unload_service_name = '/controller_manager/unload_controller'
        self.unload_service = rospy.ServiceProxy(self.unload_service_name, UnloadController)

        self.vel_controller = [ "joint_state_controller",
                                "gripper_controller",
                                "ur_shoulder_pan_vel_controller",
                                "ur_shoulder_lift_vel_controller",
                                "ur_elbow_vel_controller",
                                "ur_wrist_1_vel_controller",
                                "ur_wrist_2_vel_controller",
                                "ur_wrist_3_vel_controller"]
        self.vel_traj_controller = ['joint_state_controller',
                                    'gripper_controller',
                                    'vel_traj_controller']
     

    def switch_controllers(self, controllers_on, controllers_off, strictness=1):
        """
        Give the controllers you wan to switch on or off.
        :param controllers_on: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :param controllers_off: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        rospy.wait_for_service(self.switch_service_name)

        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.stop_controllers = controllers_off
            switch_request_object.strictness = strictness

            switch_result = self.switch_service(switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            rospy.logdebug("Switch Result==>"+str(switch_result.ok))

            return switch_result.ok

        except rospy.ServiceException, e:
            print (self.switch_service_name + " service call failed")

            return None

    def stop_controllers(self, controllers_off, strictness=1):
        """
        Give the controllers you wan to stop.
        :param controllers_off: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        rospy.wait_for_service(self.switch_service_name)

        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = []
            switch_request_object.stop_controllers = controllers_off
            switch_request_object.strictness = strictness

            switch_result = self.switch_service(switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            rospy.logdebug("Switch Result==>"+str(switch_result.ok))

            return switch_result.ok

        except rospy.ServiceException, e:
            print (self.switch_service_name + " service call failed")

            return None
    
    def stop_all_controller(self,  strictness=1):
        """
        Give the controllers you wan to stop.
        :param controllers_off: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        rospy.wait_for_service(self.switch_service_name)

        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = []
            switch_request_object.stop_controllers = [self.vel_traj_controller, self.vel_controller]
            switch_request_object.strictness = strictness

            switch_result = self.switch_service(switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            rospy.logdebug("Switch Result==>"+str(switch_result.ok))

            return switch_result.ok

        except rospy.ServiceException, e:
            print (self.switch_service_name + " service call failed")

            return None

    def start_controllers(self, controllers_on, strictness=1):
        """
        Give the controllers you wan to stop.
        :param controllers_off: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        rospy.wait_for_service(self.switch_service_name)

        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.stop_controllers = []
            switch_request_object.strictness = strictness

            switch_result = self.switch_service(switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            rospy.logdebug("Switch Result==>"+str(switch_result.ok))

            return switch_result.ok

        except rospy.ServiceException, e:
            print (self.switch_service_name + " service call failed")

            return None

    def reset_ur_joint_controllers(self, ctrl_type):
        
        if ctrl_type == 'traj_vel':
            controllers_reset = self.vel_traj_controller
        elif ctrl_type == 'vel':
            controllers_reset = self.vel_controller

        self.reset_controllers(controllers_reset)

    def reset_controllers(self, controllers_reset):
        """
        We turn on and off the given controllers
        :param controllers_reset: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        reset_result = False

        result_off_ok = self.switch_controllers(controllers_on = [],
                                controllers_off = controllers_reset)

        if result_off_ok:
            result_on_ok = self.switch_controllers(controllers_on=controllers_reset,
                                                    controllers_off=[])
            if result_on_ok:
                rospy.logdebug("Controllers Reseted==>"+str(controllers_reset))
                reset_result = True
            else:
                rospy.logdebug("result_on_ok==>" + str(result_on_ok))
        else:
            rospy.logdebug("result_off_ok==>" + str(result_off_ok))

        return reset_result

    def load_controllers(self, load_controllers_name):
        rospy.wait_for_service(self.load_service_name)

        try:
            load_request_object = LoadControllerRequest()
            load_request_object.name = load_controllers_name

            load_result = self.load_service(load_request_object)
            """
            [controller_manager_msgs/LoadController]
            string name
            ---
            bool ok
            """
            rospy.logdebug("Load Result==>"+str(load_result.ok))

            return load_result.ok

        except rospy.ServiceException, e:
            print (self.load_service_name + " service call failed")

            return None

    def unload_controllers(self, unload_controllers_name):
        rospy.wait_for_service(self.unload_service_name)

        try:
            unload_request_object = UnloadControllerRequest()
            unload_request_object.name = unload_controllers_name

            unload_result = self.unload_service(unload_request_object)
            """
            [controller_manager_msgs/UnloadController]
            string name
            ---
            bool ok
            """
            rospy.logdebug("Load Result==>"+str(unload_result.ok))

            return unload_result.ok

        except rospy.ServiceException, e:
            print (self.unload_service_name + " service call failed")

            return None


         