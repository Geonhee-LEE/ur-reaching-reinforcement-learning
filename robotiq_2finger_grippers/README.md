This package contains the necesary files to connect and control the Robotiq 2 finger adaptive grippers (85mm and 140mm stroke) of the `C` series through a **USB port using the Modbus RTU communication protocol**. (Currently only URDF descriptions of models C3 are available)

![C3 models](https://user-images.githubusercontent.com/8356912/52115661-78872b00-260f-11e9-8eb3-960747131df6.jpg)

Note: This package is based on [waypointrobotics/robotiq_85_gripper](https://github.com/waypointrobotics/robotiq_85_gripper) and [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq), with many code changes and feature inclusions as:
  
  - Use of ros `actionlib` Action Server/Clients to properly connect, control and command the grippers.
  - Gripper driver joint position update to `/joint_state` topic.
  - Simulation of grippers operation.
  - Configuration of collision meshes as simplified convex hulls of the visual meshes for both 85mm and 140mm stroke grippers.
  - Helper functions and examples for python and C++.

*A pull request to this packages requires further analysis since I performed many structural changes, which will create conflict for previous users.*
### Ros Distro
This package has been tested on `Kinetic` and `Melodic`.
# Contents
- [Contents](#contents)
- [Operation Instructions](#operation-instructions)
  - [Serial Port Configuration](#serial-port-configuration)
  - [Test communication and control](#test-communication-and-control)
- [Documentation](#documentation)
  - [Control Robotiq gripper/s](#control-robotiq-grippers)
    - [Node `robotiq_2f_action_server`](#node-robotiq2factionserver)
      - [Arguments:](#arguments)
      - [Action type:](#action-type)
    - [Command gripper through an instance of a `SimpleActionClient` in Python](#command-gripper-through-an-instance-of-a-simpleactionclient-in-python)
    - [Command gripper through an instance of a `SimpleActionClient` in C++](#command-gripper-through-an-instance-of-a-simpleactionclient-in-c)
  - [Operation of Multiple Grippers](#operation-of-multiple-grippers)
# Operation Instructions

Prior to executing control of the grippers make sure you have connected a [_2-finger adaptive gripper model_](https://robotiq.com/support/2-finger-adaptive-robot-gripper) to a USB port of your computer. The RS-485 to USB converter _ACC-ADT-USB-RS485_ that comes by default when you order these grippers allow connecting the 2-finger gripper directly to a computer through a USB 2.0 port using Modbus RTU communication protocol.
## Serial Port Configuration

To control the gripper over a serial port, you may need to give proper privileges to the user:
```
sudo adduser <YOUR_USERNAME> dialout
```
To find out the port on which the controller is connected, use:
```
dmesg | grep tty
```
The output should look something similar to:
```
[####.#####] USB 1-2: FTDI USB Serial Device converter now attached to ttyUSB0
```
## Test communication and control 

On a new terminal type the following line:
```
# Connect and move 85 mm gripper  
roslaunch robotiq_2f_gripper_control test_85mm_gripper.launch comport:="<YOUR GRIPPER TTY PORT NAME>"
# Simulate 85mm gripper 
roslaunch robotiq_2f_gripper_control test_85mm_gripper.launch sim:=true
```
This launch file starts a `robotiq_action_server` and an example action client which recursively commands the the real gripper *or simulated one) with diferent poses/speeds while updating the gripper joints on `rviz`. 

![gripper operation](https://user-images.githubusercontent.com/8356912/52121064-a0ca5600-261e-11e9-8ad1-6b2855f11909.gif)

# Documentation
## Control Robotiq gripper/s
 The control of both 140mm and 85mm stroke robotiq grippers is done though an `actionlib` client-server interaction (see actionlib [documentation](http://wiki.ros.org/actionlib)).

![robotiq_actionlib_structure](https://user-images.githubusercontent.com/8356912/50008462-8d065a00-ffb4-11e8-931b-d686bf16c414.png)
***
### Node `robotiq_2f_action_server`
This node creates an instance of an action server that will handle the gripper connection, initialization, joint state publishing, and control by listening for incoming goals on the action name `command_robotiq_action`. It offers also the option to simulate a gripper response to incoming goals.  

#### Arguments: 
- `sim`: Whether to use a simulated gripper or not
- `comport`: Communication port to which the gripper is connected.
- `baud`: Communication baud rate (must be the same baud rate as configured on the gripper)
- `stroke`: Stroke of the gripper (maximum distance between fingers) currently only 0.085m or 0.140m values are supported.
- `joint_name`: Name of the gripper driver joint as defined in your robot URDF description. This parameter is used to publish the state of the gripper joint on the `/joint_states` topic. Default is `finger_joint`.
- `rate`: Frequency in Hz to send/request of data to the gripper and update of joint state.

#### Action type:
 [`CommandRobotiqGripper.action`](https://github.com/Danfoa/robotiq_2finger_grippers/blob/master/robotiq_2f_gripper_msgs/action/CommandRobotiqGripper.action) (see action declaration for `goal`, `feedback` and `result` message fields)
 
 or 
 
 [`FollowJointTrajectory.action`](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html) (see action declaration for `goal`, `feedback` and `result` message fields): This is the default control scheme from MoveIt!, meaning that with the appropiate configuration you can plan and command the gripper through the MoveIt!-Rviz GUI, and through Pick and Place operations.

Note: The Action Server will only become active and ready after Robotiq gripper has been propertly initialized (see robotiq documentation).

***
### Command gripper through an instance of a `SimpleActionClient` in Python
An example action client implementation is provided on the script [`robotiq_2f_action_client_example.py`](https://github.com/Danfoa/robotiq_2finger_grippers/blob/master/robotiq_2f_gripper_control/scripts/robotiq_2f_action_client_example.py), showing how to initialize the client, create custom goals, and use predefined functions for fast control of the gripper, position, speed and force. But basically you just need to:

Import the Action messages and actionlib library
```
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
```
Instantiate the action client
```
action_name = 'command_robotiq_action'    # Use your action server namespace if necesary
# Create an action client
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)   
# Wait until grippers are ready to take command (communication is stablished and gripper is activated)
robotiq_client.wait_for_server()   
```
Command your robotiq gripper
```
# Use pre-defined functions for robot gripper manipulation.
Robotiq.close(robotiq_client, block=True)   # Close and wait until completion
Robotiq.open(robotiq_client, block=False)   # Open and do not block thread while completing goal
Robotiq.goto(robotiq_client, pos=0.04, speed=0.1, force=10, block=True)  # Send command Pose[m], speed[m/s], force [%]
Robotiq.emergency_release(robotiq_client)   # Slowly open gripper and deactivate it
```
### Command gripper through an instance of a `SimpleActionClient` in C++
Import the Action messages and actionlib library
```
#include <actionlib/client/simple_action_client.h>
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

typedef robotiq_2f_gripper_control::RobotiqActionClient RobotiqActionClient;
```
Instantiate a `RobotiqActionClient` class
```
std::string action_name = "/command_robotiq_action";  
bool wait_for_server = true;                  // Wait for Action Server connection (gripper is ready and activated)
RobotiqActionClient gripper = new RobotiqActionClient(action_name, wait_for_server);
```
Command your robotiq gripper
```
gripper.close(speed, force, false);    // Close and do not wait
gripper.close();                       // Block thread until gripper is closed.
gripper.open();
gripper.goToPosition( position, speed, force, true);
```
***
## Operation of Multiple Grippers
To control multiple grippers simultaneously you need to correctly namespace the grippers Action service and client nodes. A correct example is provided in the [`robotiq_dual_action_server.launch`](https://github.com/Danfoa/robotiq_2finger_grippers/blob/master/robotiq_2f_gripper_control/launch/robotiq_dual_action_server.launch) file. 

