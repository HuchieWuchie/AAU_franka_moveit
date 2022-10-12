# AAU_franka_moveit

By Albert Daugbjerg Christensen

Aalborg University (2022)

Robotics group

## 1. Description

This repository contains a general MoveIt package for the Franka Emika Panda used for visual reasoning at the AAU robotics lab. The package can then be run on a dedicated PC, used only for communicating with the Franka robot through MoveIt. Other ROS nodes can then communicate with the Franka PC through ROS interfaces.

This readme file refers to 3 different pc's.

Franka pc: The pc for which this code is intended to run. This pc needs to be connected with an ethernet cable to the Franka controller.

Interface pc: A pc of any OS that is connected to the Franka robot ethernet port. This pc can acccess the control panel at robot.franka.de

ROS pc: The pc for where all the other computations takes place such as visual reasoning, handover tasks, etc.

## 2. Installation of pre-requisties on the Franka pc (Skip if you have the robolab pc):

It is recommended to just use the Robolab pc, if that is not possible, then follow these installation instructions.
These instructions are for installing the neccessary packages on the Franka pc, NOT the ROS pc.

### 2.1 General system requirements:
```
PC with ethernet port
Ubuntu 18.04 with real-time patch, instructions for how to install RT patch follows later
```

### 2.2 Install ROS melodic:

Follow the instructions for how to install ros melodic here: http://wiki.ros.org/melodic/Installation/Ubuntu

### 2.3 Build libfranka from source:

The instructions are taken from here: https://frankaemika.github.io/docs/installation_linux.html
Build libfranka from source, do not use the binary

Remove  any existing installations
```
sudo apt remove "*libfranka*"
```

Install pre-requisites
```
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
```

Clone and setup the installation folder
```
git clone --recursive https://github.com/frankaemika/libfranka # only for panda
cd libfranka
```

Create a build directory and run CMAKE
```
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```

Build a debian package and install the debian package
```
cpack -G DEB
sudo dpkg -i libfranka*.deb
```

### 2.4 Build franka_ros from source

Setup the catkin workspace
```
cd ~
mkdir -p franka_ros/src && cd franka_ros
source /opt/ros/melodic/setup.sh
catkin_init_workspace src
```

Clone franka_ros github repository
```
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
```

Change version to melodic
```
git checkout melodic-devel
```

Install missing dependencies
```
rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```

### 2.5 Install real-time patch

Setup a workspace
```
cd ~
mkdir rt_patch && cd rt_patch
```

Follow the instructions from here and install the RT patch for kernel version 5.4.19. Other patches might work as well, but bugs and problems have been encountered.

## 3. Install the AAU_franka_moveit repository

This step should be done on BOTH the Franka pc AND the ROS pc.

### 3.1 Install packages:

#### ROS packages:

```
sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-panda-moveit-config
sudo apt install ros-melodic-realsense2-description
```

#### Other packages:

```
sudo apt install python3-pip
```

### 3.2 Install python packages:

```
pip3 install rospkg
```

### 3.3 Setup the ros workspace

```
cd ~
mkdir ros_ws
mkdir ros_ws/src
cd ros_ws/src

git clone --branch melodic-devel https://github.com/frankaemika/franka_ros.git
git clone https://github.com/justagist/franka_panda_description.git
git clone --branch v0.7.1-dev https://github.com/justagist/franka_ros_interface.git
git clone https://github.com/HuchieWuchie/AAU_franka_moveit.git

cd ..
catkin_make
source devel/setup.bash
```

### 3.4 Setup the Franka pc (the one you installed all of this on):

Connect an ethernet cable from the Franka pc and to a switch.
Connect an ethernet cable from the switch to the Franka controller (not the actual robot).

```
IP: 172.16.0.1
Netmask: 255.255.255.0
```

Export ros settings
```
export ROS_IP=172.16.0.1
export ROS_MASTER_URI=http://172.16.0.1:11311
```

### 3.5 Setup the ROS pc (the one you will be running your neural networks on):


Connect an ethernet cable from the ROS pc and to the switch

```
IP: 172.16.0.3
Netmask: 255.255.255.0
```

Export ros settings
```
export ROS_IP=172.16.0.3
export ROS_MASTER_URI=http://172.16.0.1:11311
```

### 3.6 Setup the Interface pc (TODO):

## 4. Usage

For all of the usage case scenarios, remember to activate FCI on the Interface pc, by going to robot.franka.de and unlocking the motors, and then activating FCI.

### 4.1 Basic moveit package, Franka pc

Launch file, this brings up the basic moveit package
```
source devel/setup.bash
roslaunch panda_arm_moveit_config bringup.launch
```

### 4.2 Basic moveit package with moveit interface through code

This package is meant to run on what we call the Franka pc. When you are running the RT patch, it is not possible to use NVIDIA drivers, it is therefore not possible to run any neural networks. Those networks should be run on an external pc, which we call the ROS pc. The ROS pc can then interface with the Franka PC and thereby the MoveIT package through a ROS service called the moveit_service package included in this repository. The moveit_service provides some basic MoveIT functionality, and can be expanded as needed.

```
roslaunch panda_arm_moveit_config bringup_moveit.launch
```

An example of how to use this service is provided in usage_example.py run it by

```
roslaunch fh_moveit_service usage_example.py
```

## 5. Controlling the Franka robot from the ROS pc

This is todo, I will write it later.
But it is basicly just calling the various moveit services.


## 6. Further development and nice to know

The workspace is described in the ws_description package. You can add additional sensors and environmental description in the ws_description/urdf/panda_arm_hand.urdf.xacro file.

## 7. Trouble shooting

Todo

### 7.1 Reflex mode

bringup_moveit.launch or bringup.launch launches successfully, but the robot wont move, even though the planning works.

Check if you get the following error in the terminal:

"libfranka move command rejected: command not possible in the current mode ("Reflex")"
- If so, shut down the roscore, press down and unpress the Franka activation button, then restart ROS.

### 7.2 Goal tolerance violated

The robot does not execute any trajectories generated by moveit for whatever reason:

Error:

```
Controller position_joint_trajectory_controller failed with error GOAL_TOLERANCE_VIOLATED:
```

Can sometimes be fixed by either restarting the roscore or in other cases you have to restart the whole robot.
But this should probably be fixed in moveit.
