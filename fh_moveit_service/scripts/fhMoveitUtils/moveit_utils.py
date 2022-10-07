#!/usr/bin/env python3
import rospy

import moveit_msgs
import geometry_msgs
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotTrajectory

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
from fh_moveit_service.srv import moveitSetMaxAccSrv, moveitSetMaxAccSrvResponse
from fh_moveit_service.srv import moveitSetMaxVelSrv, moveitSetMaxVelSrvResponse
from fh_moveit_service.srv import moveitSetPlanningTimeSrv, moveitSetPlanningTimeSrvResponse
from fh_moveit_service.srv import moveitNumPlanningAttemptsSrv, moveitNumPlanningAttemptsSrvResponse

def setMaxAcceleratoinScalingFactor(val: float):
    rospy.wait_for_service("/fh_handover/moveit/set_max_acceleration_scaling_factor")
    service = rospy.ServiceProxy("/fh_handover/moveit/set_max_acceleration_scaling_factor", moveitSetMaxAccSrv)

    msg = moveitSetMaxAccSrv()
    msg.data = val

    service(msg)

def setMaxVelocityScalingFactor(val: float):
    rospy.wait_for_service("/fh_handover/moveit/set_max_velocity_scaling_factor")
    service = rospy.ServiceProxy("/fh_handover/moveit/set_max_velocity_scaling_factor", moveitSetMaxVelSrv)

    msg = moveitSetMaxVelSrv()
    msg.data = val

    service(msg)


def setNumPlanningAttempts(val: int):
    rospy.wait_for_service("/fh_handover/moveit/set_num_planning_attempts")
    service = rospy.ServiceProxy("/fh_handover/moveit/set_num_planning_attempts", moveitNumPlanningAttemptsSrv)

    msg = moveitNumPlanningAttemptsSrv()
    msg.data = val

    service(msg)


def setPlanningTime(val: float):
    rospy.wait_for_service("/fh_handover/moveit/set_planning_time")
    service = rospy.ServiceProxy("/fh_handover/moveit/set_planning_time", moveitSetPlanningTimeSrv)

    msg = moveitSetPlanningTimeSrv()
    msg.data = val

    service(msg)


def getRobotStateAtPose(pose_msg: geometry_msgs.msg.Pose):
    """ Input:
        pose_msg            - geometry_msgs/Pose

        Output:
        valid               - Bool, is the state valid or not
        state               - RobotState
    """

    initial_state = getCurrentState() # moveit requires a start state given

    start_pose_msg = geometry_msgs.msg.PoseStamped()
    start_pose_msg.header.frame_id = "world"
    start_pose_msg.header.stamp = rospy.Time.now()
    start_pose_msg.pose = pose_msg

    ik_request_msg = moveit_msgs.msg.PositionIKRequest()
    ik_request_msg.group_name = "panda_arm"
    ik_request_msg.robot_state = initial_state
    ik_request_msg.avoid_collisions = True #False
    ik_request_msg.pose_stamped = start_pose_msg
    ik_request_msg.timeout = rospy.Duration(1.0) #
    ik_request_msg.attempts = 10

    rospy.wait_for_service('/compute_ik')
    ik_calculator = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    state = ik_calculator(ik_request_msg)
    valid = False
    if state.error_code.val == 1:
        valid = True

    return valid, state.solution



def moveToNamed(name: str):
    """ moves robot to predefined named pose
    """

    if not len(name):
        print("ERROR: Specify a named pose")
        return False

    rospy.wait_for_service("/fh_handover/moveit/move_to_named")
    service = rospy.ServiceProxy("/fh_handover/moveit/move_to_named", moveitMoveToNamedSrv)

    msg = moveitMoveToNamedSrv()
    msg.data = name

    success = service(msg)

    return success


def executeTrajectory(trajectory: moveit_msgs.msg.RobotTrajectory):
    """ Executes a precomputed trajectory """

    rospy.wait_for_service("/fh_handover/moveit/execute")
    tf2Service = rospy.ServiceProxy("/fh_handover/moveit/execute", moveitExecuteSrv)

    msg = moveitExecuteSrv()
    msg = trajectory

    success = tf2Service(msg)

    return success

def gripperClose():

    rospy.wait_for_service("/fh_handover/moveit/gripper_close")
    service = rospy.ServiceProxy("/fh_handover/moveit/gripper_close", moveitGripperCloseSrv)

    msg = moveitGripperCloseSrv()

    success = service()

    return success

def gripperOpen():

    rospy.wait_for_service("/fh_handover/moveit/gripper_open")
    service = rospy.ServiceProxy("/fh_handover/moveit/gripper_open", moveitGripperOpenSrv)

    msg = moveitGripperOpenSrv()

    success = service()

    return success


def getCurrentState() -> moveit_msgs.msg.RobotState:

    rospy.wait_for_service("/fh_handover/moveit/getRobotState")
    service = rospy.ServiceProxy("/fh_handover/moveit/getRobotState", moveitRobotStateSrv)

    msg = moveitRobotStateSrv()
    msg.data = True

    state = service(msg).state

    return state

def planToNamed(name: str) -> moveit_msgs.msg.RobotTrajectory:
    """ Computes a trajectory for predefined named pose """

    rospy.wait_for_service("/fh_handover/moveit/plan_to_named")
    service = rospy.ServiceProxy("/fh_handover/moveit/plan_to_named", moveitPlanToNamedSrv)

    msg = moveitPlanToNamedSrv()
    msg.data = name

    response = service(msg)
    return response.plan

def planFromPoseToPose(start_pose: geometry_msgs.msg.Pose,
                    goal_pose: geometry_msgs.msg.Pose):

    """ Computes a trajectory from one pose to another pose
        Output:     success         -   Bool
                    trajectory      -   moveit_msgs.msg.RobotTrajectory
    """

    rospy.wait_for_service("/fh_handover/moveit/plan_from_pose_to_pose")
    service = rospy.ServiceProxy("/fh_handover/moveit/plan_from_pose_to_pose", moveitPlanFromPoseToPoseSrv)

    response = service(start_pose, goal_pose)
    return response.success, response.plan

def getInverseKinematicsSolution(initial_state: moveit_msgs.msg.PositionIKRequest.robot_state,
                                goal_pose: geometry_msgs.msg.Pose):
    """ ???
        Input:
        Output:
    """

    goal_pose_msg = geometry_msgs.msg.PoseStamped()
    goal_pose_msg.header.frame_id = "world"
    goal_pose_msg.header.stamp = rospy.Time.now()
    goal_pose_msg.pose = goal_pose

    ik_request_msg = moveit_msgs.msg.PositionIKRequest()
    ik_request_msg.group_name = "panda_arm"
    ik_request_msg.robot_state = initial_state
    ik_request_msg.avoid_collisions = False #False
    ik_request_msg.pose_stamped = goal_pose_msg
    ik_request_msg.timeout = rospy.Duration(0.25) #
    ik_request_msg.attempts = 5

    rospy.wait_for_service('/compute_ik')
    ik_calculator = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    state = ik_calculator(ik_request_msg)
    valid = False
    if state.error_code.val == 1:
        valid = True

    return valid, state

def planToPose(goal_pose: geometry_msgs.msg.Pose):
    """ Plans from current state to a given pose
        Output: success         -   Bool
                trajectory      -   moveit_msgs.msg.RobotTrajectory
    """

    rospy.wait_for_service("/fh_handover/moveit/plan_to_pose")
    service = rospy.ServiceProxy("/fh_handover/moveit/plan_to_pose", moveitPlanToPoseSrv)

    response = service(goal_pose)
    return response.success, response.plan

def getJointPositionAtNamed(target: str) -> list:

    rospy.wait_for_service("/fh_handover/moveit/getJointPositionAtNamed")
    service = rospy.ServiceProxy("/fh_handover/moveit/getJointPositionAtNamed", moveitGetJointPositionAtNamed)
    msg = moveitGetJointPositionAtNamed()
    msg.data = target
    response = service(msg)

    return response
