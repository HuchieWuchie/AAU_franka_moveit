Overview
===================================

The code base is
meant to be run across several PCs. We define 3 PCs.

.. code-block:: RST

    * Interface PC: A pc of any OS that is connected to the Franka robot ethernet port. This PC can acccess the control panel at robot.franka.de
    * Franka PC: This PC runs MoveIt and is in charge of communicating with the Franka robot (hence the name).
    * ROS PC: The PC for where all the other computations takes place such as visual reasoning, grasping, handover tasks, etc. This PC is also in charge of communicating with the sensors such as microphones, lidars, cameras, etc.

An overview of the 3 PCs and how they are connected can be seen in the figure below:

.. image:: images/franka_setup.png
  :width: 800
  :alt: Alternative text

As can be seen from the image above, the setup requires communication to happend
between several PCs, therefore, two github repositories has been coded to allow
for this setup:

.. note::

   https://github.com/HuchieWuchie/AAU_franka_moveit
   https://github.com/HuchieWuchie/franka_handover


The **AAU_franka_moveit** repository serves as a base that can compute 
collision free trajectories and execute said trajectories. The repository
is meant to run run on the **Franka PC**, it contains the following:

.. code-block:: RST

    * A URDF description of the workspace.
    * A MoveIt implementation.
    * A ROS service that enables moveit commands to be sent from other PCs.

The **franka_handover** repository contains a codebase for communicating with the
sensors attached to the setup, it also contains code defining a set of skills such
as object affordance detection and task-oriented grasping. The repository is meant 
to be run on the **ROS PC**, it contains the following:

.. code-block:: RST

    * AffordanceAnalyzer (Object affordance prediction)
    * fhUtils (Utility functions)
    * fh_handover (Example scripts of how to use the repository)
    * graspGenerator (Sample task-agnostic grasps)
    * handoverLocation (Node for computing the x, y, z position of the receiver)
    * handoverOrientation (Node for computing proper object orientation for handover task)
    * sensors/camera (Utility node for communicating with intel Realsense sensors)


The system is meant to be run on a multi-PC setup, where various computations
are assigned to the different units. Processing of the sensor data happens on the
**ROS PC**, which can be used to communicate where and when the physical robot 
should move. The majority of the codebase and development is therefore meant to 
happen on the **ROS PC**. Commanding the physical robot is done with the ``ROS`` 
plugin called ``MoveIt``. Commanding the physical robot utilizes the ``libfranka``
C++ plugin. ``libfranka`` requires a PC to run a real-time kernel, as such a
second PC is required, this is the **Franka PC**. The **Franka PC** will not be 
able to run any NVidia drivers or CUDA applications, since that software is not
available in real-time systems. In order to communicate commands from the 
**ROS PC** to the **Franka PC** a ROS service called ``moveitService`` has been 
developed, which can be found in the AAU_franka_moveit repository.

The rest of the documentation covers how to install and use the codebase.