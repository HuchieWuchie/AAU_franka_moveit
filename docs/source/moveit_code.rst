2 Moveit Using code
===================================

Needed equipment:
#####

    | Franka PC
    | ROS PC
    | Interface PC

Packages used
#####

From the ``AAU_franka_moveit`` repository::

    **panda_arm_moveit_config** # MoveIt configuration of the workspace
    **ws_description** # Used by MoveIt to describe the physical workspace
    **moveitService** # Used to send goal poses for the robot from the ROS PC to the 
    MoveIt package on the Franka PC. 

Description
######

This tutorial describes how we can make the movements and trajectories that we
did with the MoveIT GUI, but this time through code. We command the robot using
MoveIt as before. The commands will originate from the **ROS PC** but be executed
by the **Franka PC**. The communication between the **ROS PC** and the **Franka PC**
happens via a ``ROS service``. The service server is hosted on the **Franka PC** 
and the **ROS PC** runs the client which can make requests.

The client can be coded in either ``C++`` or ``Python``, this tutorial is ``Python``
only.

For a overview of the system in this tutorial, see figure below.

.. image:: images/moveit_code.png
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

Franka PC:
*******

Navigate to the ``AAU_franka_moveit`` workspace, wherever you have located it.

Setup the ROS network parameters::

    export ROS_IP=172.16.0.1
    export ROS_MASTER_URI=http://172.16.0.1:11311

Source the work environment::

    source devel/setup.bash

Launch the base moveit launch file::

    roslaunch panda_arm_moveit_config aau_bringup.launch

ROS PC:
********