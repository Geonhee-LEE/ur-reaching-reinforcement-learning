# Universal robot with robotiq hand workspace

## Build

```
source /opt/ros/kinetic/setup.bash
cd ur_ws/src
wstool update
cd ../
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin build
```

## Run

```
source devel/setup.bash
roslaunch ur_robotiq_gazebo ur_robotiq_gazebo.launch
```
![ur_robotiq](images/ur_robotiq.jpg)

## Grasp

The other terminal

```
rosrun ur_robotiq_manipulation grasp_object.py
```

![grasping](images/grasping.gif)

## Camera Images

RGB image

![rbg](images/camera_rgb_image.png)

Depth image

![depth](images/camera_depth_image.png)

## Multiple Arms

![multi_arms](images/multiple_arms.jpg)
