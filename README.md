# Gazebo Environments for JetBot

Gazebo environments for testing JetBot navigation and perception algorithms.

## Installation Instructions

This instructions are for Ubuntu 18.04 with ROS Melodic already installed.
Clone this repo into your catkin_ws (the code below creates a new catkin workspace named jetbot_ws in your home folder):

```
mkdir -p  ~/jetbot_ws/src && cd ~/jetbot_ws/src
git clone --recursive https://github.com/TIERS/jetbot-gezebo.git

cd ~/jetbot_ws/  && catkin build
```

Install dependencies
```
sudo apt install ros-melodic-turtlebot3-gazebo ros-melodic-teleop-twist-keyboard
```

## Run the simulator

Run the simulator with the 
```
roslaunch jetbot_gazebo jetbot_gazebo_stage.launch
```

Control the Jetbot by keyboard:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Simulation Environments

TODO

## Object Detection with Yolo and Darket-ROS

TODO

## Contact

For any questions, write to `tiers@utu.fi`.

Visit us at (https://tiers.utu.fi)[https://tiers.utu.fi]
