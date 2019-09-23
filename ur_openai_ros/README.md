# How to launch original env
 First launch the simulator
  
  ``` 
  roslaunch ur_robotiq_gazebo gym.launch
  ```
 
 And run the training launch
  ```
  roslaunch ur_training default.launch
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
