4 Grasping online
===================================

Needed equipment:
#####

    | ROS PC
    | Interface PC
    | Franka PC

From the ``AAU_franka_moveit`` repository::

    **panda_arm_moveit_config** # MoveIt configuration of the workspace
    **ws_description** # Used by MoveIt to describe the physical workspace
    **moveitService** # Used to send goal poses for the robot from the ROS PC to the MoveIt package on the Franka PC.

For a overview of the system in this tutorial, see figure below.

.. image:: images/moveit_code.png
  :width: 800
  :alt: Alternative text

Step-by-step:
######

Interface PC::
*******

Perform the following steps on the Interface PC::

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

Navigate to the ROS workspace, wherever you have located it.

Setup the ROS network parameters::

    export ROS_IP=172.16.0.3
    export ROS_MASTER_URI=http://172.16.0.1:11311

Source the work environment::

    source devel/setup.bash

The full code is in the file:

``ros_ws/src/AAU_franka_moveit/fh_moveit_service/usage_example.py``

and it can be run by running the following command::

    rosrun fh_moveit_service usage_example.py