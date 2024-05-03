# Gazebo Environments for Robotics

Gazebo environments for testing navigation and perception algorithms in Ubuntu 18.04 with ROS Melodic.

This uses the Gazebo models from [https://github.com/dusty-nv/jetbot_ros](https://github.com/dusty-nv/jetbot_ros) but does not require CUDA dependencies in `jetbot_camera`.

## ROS Melodic Installation
>*adapted from instructions provided by the [ROS Wiki](http://wiki.ros.org/melodic/Installation/Ubuntu)*

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

Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception. *You can check out the ROS Wiki for other installations. Full installation may not be required.*
```bash
sudo apt install ros-melodic-desktop-full
```

It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

***Package python-rosdep Required.*** *Unsure if any others are necessary for this project*

To install this tool and other dependencies for building ROS packages, run:
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. With the following, you can initialize rosdep.
```bash
sudo rosdep init
rosdep update
```

## Simulator Installation Instructions

Clone this repo into your catkin_ws (the code below creates a new catkin workspace named jetbot_ws in your home folder):

**TODO:** Rename project and directory structure.

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
### Setup Keyboard Control

#### Option 1

Setup control of the default model by keyboard. Open a new terminal window/tab and run:
```bash
sudo apt install ros-melodic-teleop-twist-keyboard
source ~/jetbot_ws/devel/setup.bash
```
To test control of the jetbot, run:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### Option 2
``` bash
roslaunch jetbot_gazebo teleop_keyboard.launch
```

The keyboard controls are as follows:

```
w/x:  increase/decrease linear velocity
a/d:  increase/decrease angular velocity

space key, s:  force stop
```

Press Ctrl+C to quit.

### View Camera Topic

View camera topic. Open a new terminal window/tab and run:
```bash
rosrun image_view image_view image:=image_raw
```

## Outdoor Navigation Model Testing

### Data Collection

#### Option 1
Save the collected data to the 'default' dataset
``` bash
roslaunch jetbot_gazebo data_collection.launch
```

#### Option 2
Automatically name dataset based on current time
``` bash
./launch_data_collection.sh
``` 

#### Option 3
Name your dataset directory
``` bash
roslaunch jetbot_gazebo data_collection.launch data_path:='YOUR CUSTOM NAME'
```

It's recommended to view the camera feed in Gazebo by going to `Window -> Topic Visualization -> gazebo.msgs.ImageStamped` and selecting the `/gazebo/default/jetbot/camera_link/camera/image` topic.

Drive the robot and press the `C` key to capture an image.  Then annotate that image in the pop-up window by clicking the center point of the path.  Repeat this all the way around the track.  It's important to also collect data of when the robot gets off-course (i.e. near the edges of the track, or completely off the track).  This way, the JetBot will know how to get back on track.

Press Ctrl+C when you're done collecting data to quit.

### Train Navigation Model

Navigate to the directory containing the training script and run the following command, substituting the path of your dataset:

``` bash
cd ~/jetbot_ws/src/jetbot-gazebo-sim/jetbot_gazebo/dnn
python train.py --data ~/jetbot_ws/src/jetbot-gazebo-sim/data/datasets/YOUR\ DATASET/
```

### Run Navigation Model

After training, launch the navigation model to enable the JetBot to autonomously navigate around the track:

``` bash
roslaunch jetbot_gazebo nav_model.launch model:=~/jetbot_ws/src/jetbot-gazebo-sim/data/models/YOUR\ Model\ Path/model_best.pth
```

> note:  to reset the position of the robot in the Gazebo environment, press `Ctrl+R`

## Contact

For any questions, write to `jared@ltr.dev`
