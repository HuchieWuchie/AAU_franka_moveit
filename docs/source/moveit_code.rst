2 Moveit Using code
===================================

Needed equipment:
#####

    Franka PC
    ROS PC
    Interface PC

Packages used
#####

From the ``AAU_franka_moveit`` repository::

    panda_arm_moveit_config # MoveIt configuration of the workspace
    ws_description # Used by MoveIt to describe the physical workspace

Description
######

This tutorial briefly describe how to launch the barebone MoveIt pacakge that 
allows us to control the Franka Robot, for which the rest of the codebase builds
upon. It also launches an RVIZ gui which allows us to make some simple motion 
commands.

We make use of two packages contained in the ``AAU_franka_moveit`` repository. The
``ws_description`` contains various Xacro and URDF files which describes the
workspace, and the relative transformations between the various objects. Anything
not described here will not be known by the robot, and as such it might collide
with whatever is not described. The ``panda_arm_moveit_config`` can go to various
poses which can be predefined or defined here. The package can then compute a
collision free trajectory and command the robot to execute said trajectory.

For a overview of the system in this tutorial, see figure below.

.. image:: images/moveit_gui.png
  :width: 800
  :alt: Alternative text

Step-by-step:
######

Interface PC::
*******

.. codeblock::

   1. Connect to `robot.franka.de`
   2. Unlock brakes
   3. Activate FCI

ROS PC:
*******

Navigate to the ``AAU_franka_moveit`` workspace, wherever you have located it.

Setup the ROS network parameters::

    export ROS_IP=172.16.0.1
    export ROS_MASTER_URI=http://172.16.0.1:11311

Source the work environment::

    source devel/setup.bash

Launch the base moveit launch file::

    roslaunch panda_arm_moveit_config bringup.launch

You can now play around with moving the robot, remember to always use "plan and execute", otherwise it won't work for whatever reason.