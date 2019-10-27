/*
    BSD 2-Clause License

    Copyright (c) 2019, Daniel Ordonez
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 * File name: robotiq_gripper_client.h
 * 
 * Description: 
 *         This header defines the `RobotiqActionClient` class which allows the user to connect and
 *         control to the gripper action server in a higher level of abstraction. 
 *          
 *
 * Author: 
 *        Daniel Felipe Ordonez Apraez - daniels.ordonez@gmail.com
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// Robotiq msgs 
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperFeedback.h> 
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperResult.h> 
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperGoal.h>

typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperGoal     CommandRobotiqGripperGoal;
typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperResult   CommandRobotiqGripperResult;
typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperAction   CommandRobotiqGripperAction;
typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback CommandRobotiqGripperFeedback;

namespace robotiq_2f_gripper_control{

    class RobotiqActionClient : actionlib::SimpleActionClient<CommandRobotiqGripperAction>{

        private:
        CommandRobotiqGripperGoal goal;

        public:
        RobotiqActionClient(std::string action_name, bool wait_for_server = true)
            : actionlib::SimpleActionClient<CommandRobotiqGripperAction>(action_name, true){
            if( wait_for_server ){
                waitForServer( ros::Duration(10) );
                if( !isServerConnected() )
                    ROS_ERROR( "Robotiq Action Server (%s) seems not to be running", action_name.c_str() );
            }
        }
        
        void close(float speed = 0.1, float force = 220, bool wait  = true){
            goal.position = 0.0;
            goal.speed = speed;
            goal.force = force;
            goal.emergency_release = false;
            if( wait )
                sendGoalAndWait( goal );
            else
                sendGoal( goal );
        }

        void open(float speed = 0.1, float force = 220, bool wait  = true){
            goal.position = 255;
            goal.speed = speed;
            goal.force = force;
            goal.emergency_release = false;
            if( wait )
                sendGoalAndWait( goal );
            else
                sendGoal( goal );
        }

        void open(bool wait){
            goal.position = 255;
            goal.speed = 0.01;
            goal.force = 10;
            goal.emergency_release = false;
            if( wait )
                sendGoalAndWait( goal );
            else
                sendGoal( goal );
        }

        void goToPosition(float position, float speed = 0.1, float force = 220, bool wait  = true){
            goal.position = position;
            goal.speed = speed;
            goal.force = force;
            goal.emergency_release = false;
            if( wait )
                sendGoalAndWait( goal );
            else
                sendGoal( goal );
        }

    };

}