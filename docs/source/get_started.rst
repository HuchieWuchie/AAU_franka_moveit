Installation
===================================

This section covers how to install and setup the **Interface PC**, **ROS PC** and 
the **Franka PC**

Requirements
-----------

.. code-block:: RST

    * Interface PC: Any pc with an ethernet port
    * ROS PC: Ethernet port and capable NVidia GPU (preferably 8 GB VRAM)
     should run Ubuntu 18.04
    * Franka PC: Ethernet port, Ubuntu with PREEMPT_RT patched kernel

Connecting the hardware
----------------------

Start by connecting all the hardware together as shown in the the diagram.

.. image:: images/franka_setup.png
  :width: 800
  :alt: Alternative text

.. code-block:: RST

    * Connect the Interface PC to the base of the robot
    * Connect the Franka controller to the robot, look at Franka manual
    * Connect a network switch to the Franka controller
    * Connect the Franka PC to the network switch
    * Connect the ROS PC to the network switch 
    * Connect peripherals such as sensors (Lidar, camera, etc.) to ROS pc.

Interface PC: Installation
--------

Nothing is required, you can check that the connection to the robot works by
accessing robot.franka.de from either Chrome or Firefox.

You can access the robot with the following login information:

.. note::
    | Username: Panda
    | Password: panda1234

Franka PC: Installation
---------

NOTE!!! THE TUTORIAL IS WRITTEN FOR **UBUNTU 18.04** BUT 20.04 CAN ALSO WORK, BUT 
YOU HAVE TO MAKE THE NECCESSARY CHANGES YOURSELF.

Install ROS melodic:
#########

Follow the instructions for how to install ros melodic here: http://wiki.ros.org/melodic/Installation/Ubuntu

Build libfranka from source:
#########

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

Build franka_ros from source:
#########

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

Install real-time patch:
#########

Setup a workspace
```
cd ~
mkdir rt_patch && cd rt_patch
```

Follow the instructions from here and install the RT patch for kernel version 5.4.19. Other patches might work as well, but bugs and problems have been encountered.

Install the AAU_franka_moveit repository:
#########

After having installed the pre-requisites on the Franka pc, the moveit package given in this repository needs to be installed. Notice this package also needs to be installed on the ROS pc since it contains some service messages that is required for communicating between the Franka and ROS pc, but that is described further later on.

**Install packages:**

Install the following ros packages.

.. code-block:: console
    sudo apt install ros-melodic-moveit
    sudo apt install ros-melodic-panda-moveit-config
    sudo apt install ros-melodic-realsense2-description


Install the following additional packages

.. code-block:: console
    sudo apt install python3-pip

Install the following python packges

.. code-block:: console
    pip3 install rospkg

To use Lumache, first install it using pip:

.. code-block:: console

   (.venv) $ pip install lumache

Setup the ros workspace::

    (.venv) $ pip install lumache
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





