#!/usr/bin/env python3
import rospy


import moveit_msgs
import geometry_msgs
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotTrajectory

import fhMoveitUtils.moveit_utils as moveit

from fh_moveit_service.srv import moveitMoveToNamedSrv, moveitMoveToNamedSrvResponse
from fh_moveit_service.srv import moveitPlanToNamedSrv, moveitPlanToNamedSrvResponse
from fh_moveit_service.srv import moveitPlanFromPoseToPoseSrv, moveitPlanFromPoseToPoseSrvResponse
from fh_moveit_service.srv import moveitMoveToPoseSrv, moveitMoveToPoseSrvResponse
from fh_moveit_service.srv import moveitExecuteSrv, moveitExecuteSrvResponse
from fh_moveit_service.srv import moveitRobotStateSrv, moveitRobotStateSrvResponse
from fh_moveit_service.srv import moveitPlanToPoseSrv, moveitPlanToPoseSrvResponse
from fh_moveit_service.srv import moveitGetJointPositionAtNamed, moveitGetJointPositionAtNamedResponse
from fh_moveit_service.srv import moveitGripperCloseSrv, moveitGripperCloseSrvResponse
from fh_moveit_service.srv import moveitGripperOpenSrv, moveitGripperOpenSrvResponse

if __name__ == "__main__":

    rospy.init_node('aau_moveit_usage_example', anonymous=True)

    print(moveit.getCurrentState())

    # Set planning parameters
    moveit.setMaxVelocityScalingFactor(0.2)
    moveit.setMaxAcceleratoinScalingFactor(0.2)
    moveit.setPlanningTime(5.0)
    moveit.setNumPlanningAttempts(50)

    # Control the gripper in a binary fashion
    moveit.gripperClose()
    moveit.gripperOpen()

    # Move the robot using pre-defined poses
    moveit.moveToNamed("ready")
    moveit.moveToNamed("camera_ready_1")
    moveit.moveToNamed("ready")
    moveit.moveToNamed("home")

    # Plan a trajectory to a pose given as consisting of a position and a quaternion

    ## Define a pose (ROS message)

    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0.299
    pose.position.y = 0.01
    pose.position.z = 0.490

    pose.orientation.x = 0.516
    pose.orientation.y = 0.574
    pose.orientation.z = 0.445
    pose.orientation.w = -0.455

    ## Compute the trajectory and execute it

    success, trajectory = moveit.planToPose(pose)
    print("Found trajectory: ", success)

    if success:
        moveit.executeTrajectory(trajectory)

    ## Finally move back to home pose
    moveit.moveToNamed("home")
