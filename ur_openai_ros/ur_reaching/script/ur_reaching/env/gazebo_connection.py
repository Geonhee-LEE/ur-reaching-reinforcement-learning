import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

class GazeboConnection():
    
    def __init__(self):
        print ("GazeboConnection initialization !!!!!")
        
        self._init_values = rospy.ServiceProxy('/init_values', SetBool)
        self._adjust_gravity = rospy.ServiceProxy('/adjust_gravity', SetBool)
        self._change_gravity_zero = rospy.ServiceProxy('/change_gravity_zero', SetBool)


        self._unpause = rospy.ServiceProxy('/unpause_physics', SetBool)
        self._pause = rospy.ServiceProxy('/pause_physics', SetBool)
        self._reset_simulation_proxy = rospy.ServiceProxy('/reset_simulation', SetBool)
        self._reset_world_proxy = rospy.ServiceProxy('/reset_world', SetBool)
        
        # We always pause the simulation, important for legged robots learning
        #self.pauseSim()
        print ("End GazeboConnection initialization !!!!!")

        
    def unpauseSim(self):
        rospy.wait_for_service('/unpause_physics')
        try:
            response= self._unpause(True)
            #print ("/unpauseSim: ", response.success, response.message)
        except rospy.ServiceException as e:
            print ("/unpause_physics service call failed")
       
    def pauseSim(self):
        rospy.wait_for_service('/pause_physics')
        try:
            response= self._pause(True)
            #print ("/pauseSim: ", response.success, response.message)
        except rospy.ServiceException as e:
            print ("/pause_physics service call failed")

    def resetSim(self):
        rospy.wait_for_service('/reset_simulation')
        try:
            response= self._reset_simulation_proxy(True)
            #print ("/resetSim: ", response.success, response.message)
        except rospy.ServiceException as e:
            print ("/reset_simulation service call failed")

    def resetWorld(self):
        rospy.wait_for_service('/reset_world')
        try:
            response= self._reset_world_proxy(True)
            #print ("/resetWorld: ", response.success, response.message)
        except rospy.ServiceException as e:
            print ("/reset_world service call failed")

    def init_values(self):
        rospy.wait_for_service('/init_values')
        try:
            response= self._init_values(True)
            #print ("/init_values: ", response.success, response.message)
        except rospy.ServiceException as e:
            print ("/init_values service call failed")

    def adjust_gravity(self):
        rospy.wait_for_service('/adjust_gravity')
        try:
            response= self._adjust_gravity(True)
            #print ("/adjust_gravity: ", response.success, response.message)
        except rospy.ServiceException as e:
            print ("/adjust_gravity service call failed")

    def change_gravity_zero(self):
        rospy.wait_for_service('/change_gravity_zero')
        try:
            response= self._change_gravity_zero(True)
            #print ("/change_gravity_zero: ", response.success, response.message)
        except rospy.ServiceException as e:
            print ("/change_gravity_zero service call failed")