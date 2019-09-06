#!/usr/bin/env python
import time
import rospy
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty

"""
user:/opt/ros/indigo/share/gazebo_msgs/srv$ cat SetPhysicsProperties.srv
# sets pose and twist of a link.  All children link poses/twists of the URDF tree will be updated accordingly
float64 time_step                  # dt in seconds
float64 max_update_rate            # throttle maximum physics update rate
geometry_msgs/Vector3 gravity      # gravity vector (e.g. earth ~[0,0,-9.81])
gazebo_msgs/ODEPhysics ode_config  # configurations for ODE
---
bool success                       # return true if set wrench successful
string status_message              # comments if available
"""


class GravityControl(object):
    def __init__(self):

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        service_name = '/gazebo/set_physics_properties'
        rospy.loginfo("Waiting for service " + str(service_name))
        rospy.wait_for_service(service_name)
        rospy.loginfo("Service Found " + str(service_name))

        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)

        self.init_values()

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

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")

        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self._time_step.data
        set_physics_request.max_update_rate = self._max_update_rate.data
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config

        print str(set_physics_request.gravity)

        result = self.set_physics(set_physics_request)
        rospy.loginfo("Gravity Update Result==" + str(result.success) + ",message==" + str(result.status_message))

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")

    def change_gravity(self, x, y, z):
        self._gravity.x = x
        self._gravity.y = y
        self._gravity.z = z

        self.update_gravity_call()


def loop():
    rospy.init_node('change_gravity')

    gravity = GravityControl()

    rate = rospy.Rate(1)
    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True
        x = 0
        y = 0
        z = 0
        gravity.change_gravity(x, y, z)

    rospy.on_shutdown(shutdownhook)

    print "Start Moving"

    x = 0
    y = 0
    z = 5.0
    while not ctrl_c:
        rospy.loginfo("Changing Gravity Now " + str(z))
        gravity.change_gravity(x, y, z)
        rate.sleep()
        z *= -1


def sequence():
    rospy.init_node('change_gravity')

    gravity = GravityControl()
    gravity_value = 1.0

    x = 0.0
    y = -gravity_value
    z = - gravity_value / 1.0

    rospy.loginfo("Y - ")
    gravity.change_gravity(x, y, z)
    time.sleep(1)

    x = 0.0
    y = 0.0
    z = - gravity_value / 1.0

    rospy.loginfo("STOP ")
    gravity.change_gravity(x, y, z)
    time.sleep(1)

    x = 0.0
    y = gravity_value
    z = - gravity_value / 1.0

    rospy.loginfo("Y + ")
    gravity.change_gravity(x, y, z)
    time.sleep(2)

    x = 0.0
    y = 0.0
    z = - gravity_value / 1.0

    rospy.loginfo("STOP ")
    gravity.change_gravity(x, y, z)
    time.sleep(1)

    x = gravity_value
    y = 0.0
    z = - gravity_value / 1.0

    rospy.loginfo("X + ")
    gravity.change_gravity(x, y, z)
    time.sleep(1)

    x = 0.0
    y = 0.0
    z = - gravity_value / 1.0

    rospy.loginfo("STOP ")
    gravity.change_gravity(x, y, z)
    time.sleep(1)

    x = -gravity_value
    y = 0.0
    z = - gravity_value / 1.0

    rospy.loginfo("X -")
    gravity.change_gravity(x, y, z)
    time.sleep(2)

    x = 0.0
    y = 0.0
    z = - gravity_value / 1.0

    rospy.loginfo("STOP ")
    gravity.change_gravity(x, y, z)
    time.sleep(1)

    x = 0.0
    y = 0.0
    z = + gravity_value / 1.0

    rospy.loginfo("X -")
    gravity.change_gravity(x, y, z)
    time.sleep(2)

    x = 0.0
    y = 0.0
    z = - gravity_value / 1.0

    rospy.loginfo("STOP ")
    gravity.change_gravity(x, y, z)
    time.sleep(1)


if __name__ == '__main__':
    sequence()


