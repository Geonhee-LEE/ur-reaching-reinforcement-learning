#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3


# Solve conda virtual env(gazebo call error) 
class GazeboExecution():
    
    def __init__(self):
        rospy.init_node('gazebo_connection')
        print ("GazeboExcution initailization !!!!!")
        # Gazebo reset service server
        init_values = rospy.Service('/init_values', SetBool, self._init_values)
        adjust_gravity_server = rospy.Service('/adjust_gravity', SetBool, self._adjust_gravity)
        change_gravity_zero_server = rospy.Service('/change_gravity_zero', SetBool, self._change_gravity_zero)
        unpause_server = rospy.Service('/unpause_physics', SetBool, self._unpause_physics)
        pause_server = rospy.Service('/pause_physics', SetBool, self._pause_physics)
        reset_server = rospy.Service('/reset_simulation', SetBool, self._reset_simulation)
        reset_world_server = rospy.Service('/reset_world', SetBool, self._reset_world)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        # Setup the Gravity Controle system
        service_name = '/gazebo/set_physics_properties'
        rospy.logdebug("Waiting for service " + str(service_name))
        rospy.wait_for_service(service_name)
        rospy.logdebug("Service Found " + str(service_name))

        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.init_values()
        #self.pauseSim()
        
    def _init_values(self, req):
        self.init_values()
        return SetBoolResponse(True, "_init_values")
    
    def _adjust_gravity(self, req):
        self._gravity.x = 0
        self._gravity.y = 0
        self._gravity.z = -9.81
        self.update_gravity_call()
        return SetBoolResponse(True, "_adjust_gravity")

    def _change_gravity_zero(self, req):
        self._gravity.x = 0
        self._gravity.y = 0
        self._gravity.z = -0.0
        self.update_gravity_call()
        return SetBoolResponse(True, "_change_gravity_zero")
        
    def _reset_simulation(self, req):
        self.resetSim()
        return SetBoolResponse(True, "_reset_simulation")

    def _reset_world(self, req):
        self.resetWorld()
        return SetBoolResponse(True, "_reset_world")


    def _unpause_physics(self, req):
        self.unpauseSim()
        return SetBoolResponse(True, "_unpause_physics")
    
    def _pause_physics(self, req):
        self.pauseSim()
        return SetBoolResponse(True, "_pause_physics")

    def pauseSim(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")
        
    def unpauseSim(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")
        
    def resetSim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_simulation service call failed")

    def resetWorld(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_world service call failed")

    def init_values(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            # reset_proxy.call()
            self.reset_proxy()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_simulation service call failed")

        self._time_step = Float64(0.001)
        self._max_update_rate = Float64(1000.0)

        self._gravity = Vector3()
        self._gravity.x = 0.0
        self._gravity.y = 0.0
        self._gravity.z = 0.0

        self._ode_config = ODEPhysics()
        self._ode_config.auto_disable_bodies = False
        self._ode_config.sor_pgs_precon_iters = 0
        self._ode_config.sor_pgs_iters = 50
        self._ode_config.sor_pgs_w = 1.3
        self._ode_config.sor_pgs_rms_error_tol = 0.0
        self._ode_config.contact_surface_layer = 0.001
        self._ode_config.contact_max_correcting_vel = 0.0
        self._ode_config.cfm = 0.0
        self._ode_config.erp = 0.2
        self._ode_config.max_contacts = 20

        self.update_gravity_call()

    def update_gravity_call(self):
        self.pauseSim()

        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self._time_step.data
        set_physics_request.max_update_rate = self._max_update_rate.data
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config

        rospy.logdebug(str(set_physics_request.gravity))

        result = self.set_physics(set_physics_request)
        rospy.logdebug("Gravity Update Result==" + str(result.success) + ",message==" + str(result.status_message))
        
        self.unpauseSim()


gazebo_execution = GazeboExecution()
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
