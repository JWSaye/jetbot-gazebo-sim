# Gazebo Environments for JetBot

Gazebo environments for testing JetBot navigation and perception algorithms.

This uses the Gazebo models from [https://github.com/dusty-nv/jetbot_ros](https://github.com/dusty-nv/jetbot_ros) but does not require CUDA dependencies in `jetbot_camera`.

## ROS Melodic Installation

Setup your computer to accept software from packages.ros.org.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys
```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Make sure your Debian package index is up-to-date:
```bash
sudo apt update
```

Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception
```bash
sudo apt install ros-melodic-desktop-full
```

It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
```bash
sudo apt install python-rosdep
```

With the following, you can initialize rosdep.
```bash
sudo rosdep init
rosdep update
```

## Installation Instructions

This instructions are for Ubuntu 18.04 with ROS Melodic already installed.
Clone this repo into your catkin_ws (the code below creates a new catkin workspace named jetbot_ws in your home folder):

```bash
mkdir -p  ~/jetbot_ws/src && cd ~/jetbot_ws/src
git clone --recursive https://github.com/JWSaye/jetbot-gazebo-sim.git

sudo apt install python-catkin-tools
cd ~/jetbot_ws/  && catkin init && catkin build
```

Install dependencies
```bash
sudo apt install ros-melodic-turtlebot3-gazebo ros-melodic-image-view
```

## Run the simulator

Setup the environment for the simulator from the terminal with:
```bash
echo "source ~/jetbot_ws/devel/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:~/jetbot_ws/src/jetbot-gazebo-sim/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
source ~/.bashrc
```
Run the simulatation using the command:
```bash
roslaunch jetbot_gazebo jetbot_gazebo_stage.launch
```

Setup control of the Jetbot by keyboard. Open a new terminal window/tab and run:
```bash
sudo apt install ros-melodic-teleop-twist-keyboard
source ~/jetbot_ws/devel/setup.bash
```
To test control of the jetbot, run:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

View camera topic. Open a new terminal window/tab and run:
```bash
rosrun image_view image_view image:=image_raw
```

### Simulation Environments and Image View
TO DO

## Object Detection with Yolo and Darknet-ROS

TO DO

## Contact

For any questions, write to `jared@ltr.dev`
