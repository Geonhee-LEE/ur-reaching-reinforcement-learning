# Object tracking video using Reinforcement learning  

[![Object tracking video using Reinforcement learning](http://img.youtube.com/vi/A3mYJT2g-us/0.jpg)](https://youtu.be/A3mYJT2g-us?t=0s)  

# How to launch original env
 First launch the simulator
  
  ``` 
  roslaunch ur_robotiq_gazebo gym.launch
  ```
 
 And run the training launch
  ```
  roslaunch ur_training default.launch
  ```

## Conveyer GAZEBO env

First launch the gazebo and gym interface and node publishing block point.
 ```
 roslaunch ur_robotiq_gazebo conveyer_gym.launch --screen
 ```
 
 Run the RL algorithms and unpause the GAZEBO
  ```
  roslaunch ur_training default.launch
  ```
 

> Latest block's point:
``` 
rostopic echo /target_blocks_pose
```

> Total block's points:
``` 
rostopic echo /blocks_poses 
```


# How to launch REINFORCE algorithm
 First launch the simulator
  
  ``` 
roslaunch ur_robotiq_gazebo conveyer_gym.launch controller:=vel --screen gui:=false
  ```
 
 And load the parameters and launch python file for reset
  ```
roslaunch ur_reaching reinforcement.launch
  ```

 And start the learning algorithm 
  ```
python reinforcement_main.py 
  ```

 And unpause physics of GAZEBO simulator
 ```
 rosservice call /gazebo/unpause_physics "{}"
 ```



# How to launch PPO+GAE algorithm

First launch the simulator including loading the parameters and GAZEBO Excution func
```
roslaunch ur_robotiq_gazebo conveyer_gym.launch --screen gui:=false
```

 And start the learning algorithm 
 ```
  python ppo_gae_main.py
 ```


# How to use the RLkit 
[RLkit](https://github.com/vitchyr/rlkit) is reinforcement learning framework based on [rllab](https://github.com/rll/rllab)

## Run GAZEBO simulator and gazebo_execution 
First launch the simulator including loading the parameters and GAZEBO Excution func
```
roslaunch ur_robotiq_gazebo conveyer_gym.launch --screen gui:=false
```

## Training
 And start the SAC learning algorithm based on RLkit
 ```
  python rlkit_sac_main.py
 ```

 And unpause physics of GAZEBO simulator
 ```
 rosservice call /gazebo/unpause_physics "{}"
 ```
 
 After training, you may find the pickled files on the rlkit/data folder.
 
 you can easily see the results through selecting the generated folder about training like follwing:
 
 ```
 python viskit/frontend.py ../rlkit/data/SAC/SAC_2019_10_14_08_27_55_0000--s-0/
```

## Evaluation

If you want to evaluate the trained weight, you can run like following:
```
python rlkit/scripts/run_policy.py rlkit/data/SAC/SAC_2019_10_14_08_27_55_0000--s-0/params.pkl 
```


## Visualization

- Rviz
```
roslaunch ur_robotiq_moveit_config  moveit_rviz.launch 
```

- rviz_visual_tools
```
rosrun visual_tools rviz_visual_tools_demo
```

![Visualization](../images/rviz_visual_tools.png)

- moveit_visual_tools
```
rosrun visual_tools moveit_visual_tools_demo
```




## Services

- _/gazebo/unpause_physics_ ([Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
  - Unpause GAZEBO simulator for starting
  ```
  rosservice call /gazebo/unpause_physics "{}"
  ```
- _/set_velocity_controller_ ([SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html))
  -  Set velocity controllers including 6 velocity controllers after stoppint velocity_controller/JointTrajectoryController
- _/set_trajectory_velocity_controller_ ([SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html))
  -  Set velocity trajectory controller including Joint trajectory controllers after stopping velocity_controller/JointVelocityController
- _/stop_training_ ([SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html))
  -  Stop training process
- _/start_training_ ([SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html))
  -  Start training process


## Helpful function
We can check the controller manager status using following:
```
rosrun rqt_controller_manager rqt_controller_manager
```


![controller manager](../images/controller_manager.png)

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

```rosrun ur_robotiq_moveit_api ur_robotiq_moveit.py  ```

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


