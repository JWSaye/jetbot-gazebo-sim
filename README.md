# Gazebo Environments for JetBot

Gazebo environments for testing JetBot navigation and perception algorithms.

This uses the Gazebo models from [https://github.com/dusty-nv/jetbot_ros](https://github.com/dusty-nv/jetbot_ros) but does not require CUDA dependencies in `jetbot_camera`.

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
sudo apt install ros-melodic-turtlebot3-gazebo ros-melodic-image-view
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

View camera topic
```
rosrun image_view image_view image:=/jetbot/camera/image_raw
```

### Simulation Environments and Image View

![avatar](./world/gazebo_sim_imageview.png)



## Object Detection with Yolo and Darknet-ROS

TO DO

## Contact

For any questions, write to `jopequ@utu.fi` and `qingqli@utu.fi`.

Visit us at [https://tiers.utu.fi](https://tiers.utu.fi)
