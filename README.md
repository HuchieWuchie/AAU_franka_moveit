# AAU_franka_moveit

By Albert Daugbjerg Christensen

Aalborg University (2022)

Robotics group

## 1. Description

This repository contains a general MoveIt package for the Franka Emika Panda used for visual reasoning at the AAU robotics lab. The package can then be run on a dedicated PC, used only for communicating with the Franka robot through MoveIt. Other ROS nodes can then communicate with the Franka PC through ROS interfaces.

## 2. Installation of pre-requisties (Skip if you have the robolab pc):

It is recommended to just use the Robolab pc, if that is not possible, then follow these installation instructions.

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


### 3.1 Install ROS packages:

```
sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-panda-moveit-config
sudo apt install ros-melodic-realsense2-description
```

### 3.2 Install python packages:

```
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

Connect an ethernet cable between the Franka pc and the Franka controller (not the actual robot). Setup the network configuration on your ROS pc to the following:

```
IP: 172.16.0.1
Netmask: 255.255.255.0
```

Export ros settings (TODO), still old IIWA stuff
```
export ROS_IP=172.31.1.150
export ROS_MASTER_URI=http://172.31.1.150:11311
```

## 4. Usage

### 4.1 Basic moveit package

Launch file, this brings up the basic moveit package
```
source devel/setup.bash
roslaunch panda_arm_moveit_config bringup.launch
```

### 4.2 Basic moveit package with moveit interface through code

This package is meant to run on what we call the Franka pc. When you are running the RT patch, it is not possible to use the NVIDIA drivers, it is therefore not possible to run any neural networks. Those networks should be run on an external pc, which we call the ROS pc. The ROS pc can then interface with the Franka PC and thereby the MoveIT package through a ROS service called the moveit_service package included in this repository. The moveit_service provides some basic MoveIT functionality, and can be expanded as needed.

```
roslaunch panda_arm_moveit_config bringup_moveit.launch
```

An example of how to use this service is provided in todo.py

## 5. Further development and nice to know

The workspace is described in the ws_description package. You can add additional sensors and environmental description in the ws_description/urdf/panda_arm_hand.urdf.xacro file.

## 6. Trouble shooting

Todo
