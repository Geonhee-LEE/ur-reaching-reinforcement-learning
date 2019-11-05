# Universal Robot


__Usage with real Hardware__  
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

```roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.0.3```


__MoveIt! with real Hardware and gripper__  
To run the MoveIt with gripper, run
```roslaunch total_moveit_config total_moveit_planning_execution.launch ```

MoveIt API Python execution!

```roslaunch total_moveit_config total_moveit_planning_execution.launch ```

A simple test script that moves the robot to predefined positions can be executed like this:

```rosrun ur_driver test_move.py```


CAUTION:  
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.


__Usage with Gazebo Simulation__  
There are launch files available to bringup a simulated robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the simulated robot in Gazebo, run:

```roslaunch ur_gazebo ur5.launch```


__MoveIt! with a simulated robot__  
Again, you can use MoveIt! to control the simulated robot.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_gazebo ur5.launch limited:=true```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```



---


__MoveIt! with real Hardware__  
Additionally, you can use MoveIt! to control the robot.  
There exist MoveIt! configuration packages for both robots.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_bringup ur5_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


