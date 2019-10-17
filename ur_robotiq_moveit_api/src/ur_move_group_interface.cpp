
#include "ur_move_group_interface.h"
// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// Gripper
//#include <robotiq_2f_gripper_msgs/RobotiqGripperCommand.h>

//c++
#include <string>
#include <iostream>


using namespace std;
using namespace moveit::planning_interface;
using namespace moveit_visual_tools;

namespace rvt = rviz_visual_tools;

 
class URMoveGroup
{
    public:
        URMoveGroup(void)
        {
            //gripper_pub = nh_.advertise<robotiq_2f_gripper_msgs::RobotiqGripperCommand>("gripper_command", 1000);

            PLANNING_GROUP = "manipulator";

            // The :move_group_interface:`MoveGroup` class can be easily
            // setup using just the name of the planning group you would like to control and plan for.
            static MoveGroupInterface move_group(PLANNING_GROUP);
            static ::MoveItVisualTools visual_tools("base_link");

            success = false;

            init_move_group(move_group);
            init_visualization(visual_tools);
            start(move_group, visual_tools);

        }
        ~URMoveGroup(void)
        {

        }
    private:
        ros::NodeHandle nh_;
 
    public:
        std::string PLANNING_GROUP;

        // The :move_group_interface:`MoveGroup` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        //moveit::planning_interface::MoveGroupInterface move_group;

        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Raw pointers are frequently used to refer to the planning group for improved performance.
        const robot_state::JointModelGroup* joint_model_group;

        moveit_msgs::CollisionObject worktop_obj;
        moveit_msgs::CollisionObject fixed_camera_obj;
        std::vector<moveit_msgs::CollisionObject> collision_objects;

        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;

        std::string planning_frame;
        std::string endeffector_frame;

        // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
        Eigen::Affine3d _text_pose;

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan _plan;

        // Current state
        moveit::core::RobotStatePtr current_state;
        geometry_msgs::Pose current_pose;

        // Plan result; true or false
        bool success;

        // Gripper message
        //robotiq_2f_gripper_msgs::RobotiqGripperCommand gripper_cmd;
        ros::Publisher gripper_pub;

    public:
        void init_move_group(moveit::planning_interface::MoveGroupInterface& move_group) // Initialization of MoveIt group interface
        {
            planning_frame = move_group.getPlanningFrame();
            endeffector_frame = move_group.getEndEffectorLink();   
            
            // Getting Basic Information    
            ROS_INFO_STREAM("PLANNING_GROUP: " << PLANNING_GROUP);
            // We can print the name of the reference frame for this robot.
            ROS_INFO_STREAM("Planning frame: " << planning_frame);
            // We can also print the name of the end-effector link for this group.
            ROS_INFO_STREAM("End effector frame: " << endeffector_frame);

            // Raw pointers are frequently used to refer to the planning group for improved performance.
            joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

            // Create init workspace
            create_worktop(move_group.getPlanningFrame());
        }
        void init_visualization( moveit_visual_tools::MoveItVisualTools& visual_tools)  // Initialization of visualization tools
        {
            // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
            // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
            namespace rvt = rviz_visual_tools;
            visual_tools.deleteAllMarkers();

            // Remote control is an introspection tool that allows users to step through a high level script
            // via buttons and keyboard shortcuts in RViz
            visual_tools.loadRemoteControl();

            // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
            _text_pose = Eigen::Affine3d::Identity();
            _text_pose.translation().z() = 1.3;
            visual_tools.publishText( _text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
            visual_tools.trigger();
            // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
            // Start the demo
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
        }
        
        void start(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools)
        {
            // start a ROS spinning thread
            ros::AsyncSpinner spinner(1);
            spinner.start();
            
            //gripper_cmd.position = 0.5;
            //send_gripper(gripper_cmd);


            tf2::Quaternion q_orig, q_rot, q_new;
            double r=0, p=0, y=0;

            /*
            //Planning to a Pose goal
            geometry_msgs::Pose target_pose;
            // Get the original orientation of 'commanded_pose'
            tf2::convert(target_pose.orientation , q_orig);
            q_rot.setRPY(r, p, y);
            q_new.normalize();
            // Stuff the new rotation back into the pose. This requires conversion into a msg type
            tf2::convert(q_new, target_pose.orientation);
            target_pose.position.x = 0.40;
            target_pose.position.y = 0.4;
            target_pose.position.z = 0.5;
            plan_to_goal(move_group, visual_tools, target_pose);
            //Path constraint
            // Let's specify a path constraint and a pose goal for our group.
            // First define the path constraint.
            moveit_msgs::OrientationConstraint ocm;
            ocm.link_name = "tool0";
            ocm.header.frame_id = "base_link";
            ocm.orientation.w = 1.0;
            ocm.absolute_x_axis_tolerance = 0.01;
            ocm.absolute_y_axis_tolerance = 0.01;
            ocm.absolute_z_axis_tolerance = 0.01;
            ocm.weight = 0.10;
            plan_with_path_constraint(move_group, visual_tools, ocm, target_pose);
            */

            // Cartesian Paths
            update_current_state(move_group);
            geometry_msgs::Pose target_crts_pose = current_pose;
            
            // Get the original orientation of 'commanded_pose'
            tf2::convert(target_crts_pose.orientation , q_orig);
            q_rot.setRPY(r, p, y);

            q_new = q_rot*q_orig;  // Calculate the new orientation
            q_new.normalize();

            // Stuff the new rotation back into the pose. This requires conversion into a msg type
            tf2::convert(q_new, target_crts_pose.orientation);

            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_crts_pose);
            target_crts_pose.position.z -= 0.3;
            waypoints.push_back(target_crts_pose); 
            target_crts_pose.position.z += 0.3;
            waypoints.push_back(target_crts_pose); 
            target_crts_pose.position.z -= 0.4;
            waypoints.push_back(target_crts_pose);  
            target_crts_pose.position.z += 0.4;
            waypoints.push_back(target_crts_pose);  
            plan_cartesian_space(move_group, visual_tools, waypoints);

            ROS_INFO("End cartesian path1");

            update_current_state(move_group);
            std::vector<geometry_msgs::Pose> waypoints2;
            waypoints2.push_back(target_crts_pose);
            target_crts_pose.position.y += 0.4;
            waypoints2.push_back(target_crts_pose); 
            target_crts_pose.position.y -= 0.5;
            waypoints2.push_back(target_crts_pose); 
            target_crts_pose.position.y += 0.4;
            waypoints2.push_back(target_crts_pose);  
            target_crts_pose.position.y -= 0.2;
            waypoints2.push_back(target_crts_pose);  
            plan_cartesian_space(move_group, visual_tools, waypoints2);


            //Planning to a joint-space goal
            std::vector<double> target_joint_group_positions = get_current_jointgroup(move_group);
            target_joint_group_positions = get_current_jointgroup(move_group);
            target_joint_group_positions[5] = -1.57;  // radians
            plan_joint_space(move_group, target_joint_group_positions);


            //Planning to a joint-space goal
            target_joint_group_positions[5] = 0.0;  // radians
            plan_joint_space(move_group, target_joint_group_positions);


            ros::shutdown();
        }

        void create_worktop(std::string collision_id)
        {
            ROS_INFO("create_worktop");

            worktop_obj.header.frame_id = collision_id;

            // The id of the object is used to identify it.
            worktop_obj.id = "worktop";

            // Define a box to add to the world.
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.50;
            primitive.dimensions[1] = 1.0;
            primitive.dimensions[2] = 1;

            // Define a pose for the box (specified relative to frame_id)
            geometry_msgs::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.0;
            box_pose.position.y = -0.0;
            box_pose.position.z = -0.55;

            worktop_obj.primitives.push_back(primitive);
            worktop_obj.primitive_poses.push_back(box_pose);
            worktop_obj.operation = worktop_obj.ADD;

            collision_objects.push_back(worktop_obj);
                
            
            fixed_camera_obj.header.frame_id = collision_id;

            // The id of the object is used to identify it.
            fixed_camera_obj.id = "fixed_camera";

            // Define a box to add to the world.
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.08;
            primitive.dimensions[1] = 0.08;
            primitive.dimensions[2] = 1.0;

            // Define a pose for the box (specified relative to frame_id)
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.230;
            box_pose.position.y = -0.48;
            box_pose.position.z = 0.50;

            fixed_camera_obj.primitives.push_back(primitive);
            fixed_camera_obj.primitive_poses.push_back(box_pose);
            fixed_camera_obj.operation = fixed_camera_obj.ADD;

            collision_objects.push_back(fixed_camera_obj);
                
            
            // Now, let's add the collision object into the world
            ROS_INFO_NAMED("tutorial", "Add an object into the world");
            planning_scene_interface.addCollisionObjects(collision_objects);
        }

        void update_current_state(MoveGroupInterface& move_group)
        {
            // Update current state global variable
            current_state = move_group.getCurrentState();
            current_pose = move_group.getCurrentPose().pose;
        }

        std::vector<double> get_current_jointgroup(MoveGroupInterface& move_group)
        {
            // Copy joint_model_group to the join group position 
            // target_joint_group_positions[0(base_link), ..., 5(wrist3)]
            std::vector<double> target_joint_group_positions;

            //Update current state
            update_current_state(move_group);
            current_state->copyJointGroupPositions(joint_model_group, target_joint_group_positions);

            return target_joint_group_positions;
        }

        void plan_to_goal(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose target_pose)
        {
            // Planning to a Pose goal
            // We can plan a motion for this group to a desired pose for the end-effector.
            move_group.setPoseTarget(target_pose);
            // move_group.setPoseTarget(target_pose.position.x,  target_pose.position.y, target_pose.position.z, “tool0”);
            move_group.setGoalTolerance(0.1);
            
            // Now, we call the planner to compute the plan and visualize it.
            // Note that we are just planning, not asking move_group to actually move the robot.
            success = (move_group.plan(_plan) == MoveItErrorCode::SUCCESS);

            ROS_INFO_NAMED("tutorial", "Visualizing (plan to goal)  (pose goal) %s", success ? "SUCCESS" : "FAILED");

            ////// Moving to a pose goal //////
            // Moving to a pose goal is similar to the step above except we now use the move() function. Note that
            // the pose goal we had set earlier is still active and so the robot will try to move to that goal. We will
            // not use that function in this tutorial since it is a blocking function and requires a controller to be active
            // and report success on execution of a trajectory.

            /* Uncomment below line when working with a real robot */
            if(success)
                move_group.move(); 
        }

        void plan_to_goal(moveit::planning_interface::MoveGroupInterface& move_group, moveit_visual_tools::MoveItVisualTools& visual_tools, geometry_msgs::Pose target_pose)
        {
            // Planning to a Pose goal
            // We can plan a motion for this group to a desired pose for the end-effector.
            move_group.setPoseTarget(target_pose);
            move_group.setGoalTolerance(0.1);
            
            // Now, we call the planner to compute the plan and visualize it.
            // Note that we are just planning, not asking move_group to actually move the robot.
            success = (move_group.plan(_plan) == MoveItErrorCode::SUCCESS);

            ROS_INFO_NAMED("tutorial", "Visualizing (plan to goal) (pose goal) %s", success ? "SUCCESS" : "FAILED");

            // Visualizing plans
            // ^^^^^^^^^^^^^^^^^
            // We can also visualize the plan as a line with markers in RViz.
            ROS_INFO_NAMED("tutorial", "Visualizing plan (plan to goal) as trajectory line");
            visual_tools.publishAxisLabeled(target_pose, "target_pose");
            visual_tools.publishText( _text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishTrajectoryLine(_plan.trajectory_, joint_model_group);
            visual_tools.trigger();
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

            ////// Moving to a pose goal //////
            // Moving to a pose goal is similar to the step above except we now use the move() function. Note that
            // the pose goal we had set earlier is still active and so the robot will try to move to that goal. We will
            // not use that function in this tutorial since it is a blocking function and requires a controller to be active
            // and report success on execution of a trajectory.

            /* Uncomment below line when working with a real robot */
            if(success)
                move_group.move(); 
        }
                
        void plan_joint_space(MoveGroupInterface& move_group, vector<double> target_joint_group_positions)
        {
            //// Planning to a joint-space goal
            // Let's set a joint space goal and move towards it.  This will replace the pose target we set above.
            // To start, we'll create an pointer that references the current robot's state.
            // RobotState is the object that contains all the current position/velocity/acceleration data.
            
            // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
            move_group.setJointValueTarget(target_joint_group_positions);

            success = (move_group.plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing (plan joint space) (joint space goal) %s", success ? "SUCCESS" : "FAILED");

            if(success)
                move_group.move(); 
        }
        
        void plan_joint_space(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, vector<double> target_joint_group_positions)
        {
            //// Planning to a joint-space goal
            // Let's set a joint space goal and move towards it.  This will replace the pose target we set above.
            // To start, we'll create an pointer that references the current robot's state.
            // RobotState is the object that contains all the current position/velocity/acceleration data.
            
            // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
            move_group.setJointValueTarget(target_joint_group_positions);

            success = (move_group.plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing (plan joint space) (joint space goal) %s", success ? "SUCCESS" : "FAILED");

            // Visualize the plan in RViz
            visual_tools.deleteAllMarkers();
            visual_tools.publishText( _text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishTrajectoryLine(_plan.trajectory_, joint_model_group);
            visual_tools.trigger();
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the joint space demo");    

            if(success)
                move_group.move(); 
        }
                
        void plan_with_path_constraint(MoveGroupInterface& move_group,  moveit_msgs::OrientationConstraint ocm, geometry_msgs::Pose target_pose)
        {
            update_current_state(move_group);

            ////// Planning with Path Constraints
            // Path constraints can easily be specified for a link on the robot.

            // Now, set it as the path constraint for the group.
            moveit_msgs::Constraints test_constraints;
            test_constraints.orientation_constraints.push_back(ocm);
            move_group.setPathConstraints(test_constraints);

            // We will reuse the old goal that we had and plan to it.
            // Note that this will only work if the current state already satisfies the path constraints. So, we need to set the start state to a new pose.
            robot_state::RobotState start_state(*move_group.getCurrentState());

            start_state.setFromIK(joint_model_group, current_pose);
            move_group.setStartState(start_state);

            // Now we will plan to the earlier pose target from the new start state that we have just created.
            move_group.setPoseTarget(target_pose);
            move_group.setGoalTolerance(0.01);

            // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
            // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
            move_group.setPlanningTime(10.0);

            success = (move_group.plan( _plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing (plan with path constraint) (constraints) %s", success ? "SUCCESS" : "FAILED");

            // When done with the path constraint be sure to clear it.
            move_group.clearPathConstraints();

            // Since we set the start state we have to clear it before planning other paths
            move_group.setStartStateToCurrentState();

            if(success)
                move_group.move(); 
        }
                
        void plan_with_path_constraint(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, moveit_msgs::OrientationConstraint ocm, geometry_msgs::Pose target_pose)
        {    
            update_current_state(move_group);

            ////// Planning with Path Constraints
            // Path constraints can easily be specified for a link on the robot.

            // Now, set it as the path constraint for the group.
            moveit_msgs::Constraints test_constraints;
            test_constraints.orientation_constraints.push_back(ocm);
            move_group.setPathConstraints(test_constraints);

            // We will reuse the old goal that we had and plan to it.
            // Note that this will only work if the current state already satisfies the path constraints. So, we need to set the start state to a new pose.
            robot_state::RobotState start_state(*move_group.getCurrentState());
            start_state.setFromIK(joint_model_group, current_pose);
            move_group.setStartState(start_state);

            // Now we will plan to the earlier pose target from the new start state that we have just created.
            move_group.setPoseTarget(target_pose);
            move_group.setGoalTolerance(0.01);

            // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
            // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
            move_group.setPlanningTime(10.0);

            success = (move_group.plan( _plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing (plan with path constraint) (constraints) %s", success ? "SUCCESS" : "FAILED");

            // Visualize the plan in RViz
            visual_tools.deleteAllMarkers();
            visual_tools.publishAxisLabeled(current_pose, "start");
            visual_tools.publishAxisLabeled(target_pose, "goal");
            visual_tools.publishText( _text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishTrajectoryLine( _plan.trajectory_, joint_model_group);
            visual_tools.trigger();
            visual_tools.prompt("next step with path constraint");

            // When done with the path constraint be sure to clear it.
            move_group.clearPathConstraints();

            // Since we set the start state we have to clear it before planning other paths
            move_group.setStartStateToCurrentState();

            if(success)
                move_group.move(); 
        }


        void plan_cartesian_space(MoveGroupInterface& move_group, std::vector<geometry_msgs::Pose>  waypoints)
        {
            ////// Cartesian Paths
            // You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through. 
            // Note that we are starting from the new start state above.  
            // The initial pose (start state) does not need to be added to the waypoint list but adding it can help with visualizations

            // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
            // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
            // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
            move_group.setMaxVelocityScalingFactor(0.1);

            // We want the Cartesian path to be interpolated at a resolution of 1 cm
            // which is why we will specify 0.01 as the max step in Cartesian
            // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause
            // large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.1;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            // Get robot trajectory to which joint speeds will be added
            robot_trajectory::RobotTrajectory robot_move_trajectory(move_group.getCurrentState()->getRobotModel(), "manipulator");
                
            // Second get a RobotTrajectory from trajectory
            robot_move_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

            success = (move_group.plan( _plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing (plan cartesian space) (Cartesian) %s", success ? "SUCCESS" : "FAILED");

            if(success)
            {
                _plan.trajectory_ = trajectory;
                move_group.move(); 

            }

        }
                
        void plan_cartesian_space(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, std::vector<geometry_msgs::Pose>  waypoints)
        {    

            ////// Cartesian Paths
            // You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through. 
            // Note that we are starting from the new start state above.  
            // The initial pose (start state) does not need to be added to the waypoint list but adding it can help with visualizations

            // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
            // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
            // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
            move_group.setMaxVelocityScalingFactor(0.2);
            // Allow replanning
            move_group.allowReplanning(true);
            // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
            // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
            move_group.setPlanningTime(20.0);

            // We want the Cartesian path to be interpolated at a resolution of 1 cm  which is why we will specify 0.01 as the max step in Cartesian translation. 
            // We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 10;
            const double eef_step = 0.01; //[m]
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);


            // Get robot trajectory to which joint speeds will be added
            robot_trajectory::RobotTrajectory robot_move_trajectory(move_group.getCurrentState()->getRobotModel(), "manipulator");
                
            // Second get a RobotTrajectory from trajectory
            robot_move_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

            ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (plan cartesian space) (%.2f%% acheived)", fraction * 100.0);
            
                
            // Visualize the plan in RViz
            visual_tools.deleteAllMarkers();
            visual_tools.publishText(_text_pose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
            for (std::size_t i = 0; i < waypoints.size(); ++i)
                visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
            visual_tools.trigger();
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the cartesian demo");

            success = (move_group.plan( _plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian) %s", success ? "SUCCESS" : "FAILED");

            if(success)
            {
                _plan.trajectory_ = trajectory;
                move_group.execute(_plan);
            }

        }
        //void send_gripper(robotiq_2f_gripper_msgs::RobotiqGripperCommand );
};


/*
void URMoveGroup::send_gripper(robotiq_2f_gripper_msgs::RobotiqGripperCommand command)
{
    gripper_pub.publish(command);
}
*/
    
int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "ur_move_group_interface");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    //URMoveGroup ur;        
    
 
    ros::shutdown();
    return 0;
}