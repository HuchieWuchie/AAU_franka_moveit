Overview
===================================

Contents
--------

.. toctree::

   index
   usage
   api

Introduction
--------

Welcome to the **AAU Franka** documentation. This documentation covers explains
how to get started with the Franka development platform at AAU. The code base is
meant to be run across several PCs. We define 3 PCs.

.. code-block:: RST

    * Interface PC: A pc of any OS that is connected to the Franka robot ethernet port. This PC can acccess the control panel at robot.franka.de
    * Franka PC: This PC runs MoveIt and is in charge of communicating with the Franka robot (hence the name).
    * ROS PC: The PC for where all the other computations takes place such as visual reasoning, grasping, handover tasks, etc. This PC is also in charge of communicating with the sensors such as microphones, lidars, cameras, etc.

An overview of the 3 PCs and how they are connected can be seen in the figure below:

.. image:: images/franka_setup.png
  :width: 400
  :alt: Alternative text

As can be seen from the image above, the setup requires communication to happend
between several PCs, therefore, two github repositories has been coded to allow
for this setup:

.. note::

   https://github.com/HuchieWuchie/AAU_franka_moveit
   https://github.com/HuchieWuchie/franka_handover


The **AAU_franka_moveit** repository is meant as a base and will run on the
**Franka PC**, it contains the following:

.. code-block:: RST

    * A URDF description of the workspace.
    * A MoveIt implementation.
    * A ROS service that enables moveit commands to be sent from other PCs.

The **franka_handover** repository is meant as a base and will run on the
**Franka PC**, it contains the following:


**Lumache** (/lu'make/) is a Python library for cooks and food lovers
that creates recipes mixing random ingredients.
It pulls data from the `Open Food Facts database <https://world.openfoodfacts.org/>`_
and offers a *simple* and *intuitive* API.

Check out the :doc:`usage` section for further information, including
how to :ref:`installation` the project.

.. note::

   This project is under active development.
