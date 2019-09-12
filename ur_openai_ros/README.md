# How to launch original env
 First launch the simulator
  ` roslaunch ur_robotiq_gazebo gym.launch`
 
 And run the training launch
  `roslaunch ur_training  default.launch`



## Conveyer GAZEBO env

First launch the gazebo and gym interface and node publishing block point.
 `roslaunch ur_robotiq_gazebo conveyer_gym.launch --screen`
 
 Run the RL algorithms and unpause the GAZEBO
  `roslaunch ur_training default.launch`
 
