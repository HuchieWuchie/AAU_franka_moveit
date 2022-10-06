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

    moveit.setMaxVelocityScalingFactor(0.2)
    moveit.setMaxAcceleratoinScalingFactor(0.2)
    moveit.setPlanningTime(1.0)
    moveit.setNumPlanningAttempts(25)

    moveit.gripperClose()
    moveit.gripperOpen()

    moveit.moveToNamed("ready")
    moveit.moveToNamed("camera_ready_1")
    moveit.moveToNamed("ready")
    moveit.moveToNamed("home")
