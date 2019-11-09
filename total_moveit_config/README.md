__Usage with real Hardware__  
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

```roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.0.3```


__MoveIt! with real Hardware and gripper__  
To run the MoveIt with gripper, run

```roslaunch total_moveit_config total_moveit_planning_execution.launch ```

