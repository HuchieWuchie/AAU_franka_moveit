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

Modify permission for the laser scanner
```
sudo chmod a+rw /dev/ttyACM0      # note that the usb port might change
```

## 4. Usage

launch roscore and launch file
```
source devel/setup.bash
roscore
roslaunch iiwa_noPtu_moveit moveit_planning_execution.launch
```

Launch whatever experiement you want, chose between the ones listed below.
```
rosrun rob10 final_test_observation.py
rosrun rob10 final_test_rule.py
rosrun rob10 orientation_test_observation.py # user study on orientation methods
rosrun rob10 orientation_test_rule.py
rosrun rob10 orientation_test_random.py
```

In order to command the robot to pick up an object you must send a command to the rostopic /objects_affordances_id. The integer id corresponds to the object classes of AffNet-DR, eg. 1 (knife), 16 (mallet), etc.

Note if you want to run the orientation_test_METHOD.py scripts you have to make use of precomputed information which can be found at: https://drive.google.com/file/d/1OhkOdDlKzmiacBYNIeN8ccKTg_f816GE/view?usp=sharing
