# AAU_franka_moveit

By Albert Daugbjerg Christensen

Aalborg University (2022)

Robotics group

## Description

This repository contains a general MoveIt package for the Franka Emika Panda used for visual reasoning at the AAU robotics lab. The package can then be run on a dedicated PC, used only for communicating with the Franka robot through MoveIt. Other ROS nodes can then communicate with the Franka PC through ROS interfaces.

## Installation of pre-requisties (Skip if you have the robolab pc):

It is recommended to just use the Robolab pc, if that is not possible, then follow these installation instructions.

### General system requirements:
```
PC with ethernet port
Ubuntu 18.04 with real-time patch, instructions for how to install RT patch follows later
```

### Install ROS melodic:

Follow the instructions for how to install ros melodic here: http://wiki.ros.org/melodic/Installation/Ubuntu

### Install ROS packages:

```
sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-panda-moveit-config
sudo apt install ros-melodic-realsense2-description
```

### Build libfranka from source:

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

### Build franka_ros from source

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

### Install real-time patch

Setup a workspace
```
cd ~
mkdir rt_patch && cd rt_patch
```

Follow the instructions from here and install the RT patch for kernel version 5.4.19. Other patches might work as well, but bugs and problems have been encountered.



Python 3.6.9
```
open3d 0.15.2
cv2 4.2.0
numpy 1.19.5
scipy 1.5.4
scikit_learn 0.24.2
torch (Pytorch) 1.10.2 cuda version
torchvision 0.11.2 cuda
scikit_image 0.17.2
PIL 8.4.0
rospkg 1.4.0
```

The system ran on a Lenovo Thinkpad P53 laptop with a Quadro RTX 4000 GPU with 8 GB VRAM and an Intel Core i9-9880H CPU 2.3 GHZ and 32 GB RAM.


## Installation:

### Install libfranka from source

Follow the guide from here, make sure to build it from source https://frankaemika.github.io/docs/installation_linux.html

### Setup the ros workspace

```
mkdir ros_ws
mkdir ros_ws/src
cd ros_ws/src

git clone --branch melodic-devel https://github.com/frankaemika/franka_ros.git
git clone https://github.com/justagist/franka_panda_description.git
git clone --branch v0.7.1-dev https://github.com/justagist/franka_ros_interface.git
git clone this repository whatever its name is

cd ..
catkin_make
source devel/setup.bash
```

Download pretrained weights from: https://drive.google.com/file/d/1psCn_aT5KUyQDJrdxqR7GJgHeCewGokS/view?usp=sharing

Place and rename the weights file to ros_ws/src/affordanceAnalyzer/scripts/affordance_synthetic/weights.pth

## Setup of the ROS pc:

Connect an ethernet cable between the ROS pc and the KUKA sunrise controller. Setup the network configuration on your ROS pc to the following:

```
IP: 172.31.1.150
Netmask: 255.255.0.0
```

Export ros settings
```
export ROS_IP=172.31.1.150
export ROS_MASTER_URI=http://172.31.1.150:11311
```

Modify permission for the laser scanner
```
sudo chmod a+rw /dev/ttyACM0      # note that the usb port might change
```

## Usage

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
