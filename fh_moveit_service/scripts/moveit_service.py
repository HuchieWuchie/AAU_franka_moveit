#!/usr/bin/env python
import sys
import copy

import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Bool, Float64MultiArray

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

def moveToPose(req):

    print("Moving robot to cartesian pose goal: ", req.pose)
    move_group.set_pose_target(req.pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    resp = moveitMoveToPoseSrvResponse()
    resp.success.data = True

    return resp

def _planToNamed(name):
    """ Computes a trajectory for predefined named pose
        Output: trajectory      - moveit_msgs.msg.RobotTrajectory
    """

    start_state = _getCurrentState()
    robot.set_start_state(start_state)
    robot.set_named_target(name)
    trajectory = robot.plan()

    return trajectory

def planToNamed(req):

    print("Computing plan to named position: ", req.name.data)

    plan = _planToNamed(req.name.data)

    resp = moveitPlanToNamedSrvResponse()
    resp.plan = plan

    return resp

def _getRobotStateAtPose(pose_msg):
    """ Input:
        pose_msg            - geometry_msgs/Pose

        Output:
        valid               - Bool, is the state valid or not
        state               - RobotState
    """

    initial_state = getCurrentState(0) # moveit requires a start state given

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

    rospy.wait_for_service('compute_ik')
    ik_calculator = rospy.ServiceProxy("compute_ik", GetPositionIK)

    state = ik_calculator(ik_request_msg)
    valid = False
    if state.error_code.val == 1:
        valid = True

    return valid, state.solution

def planFromPoseToPose(req):

    start_pose = req.start_pose
    goal_pose = req.goal_pose

    start_pose_valid, start_state = _getRobotStateAtPose(start_pose)

    if not start_pose_valid:
        response = moveitPlanFromPoseToPoseSrvResponse()
        response.plan = RobotTrajectory()
        response.success = Bool(False)

        return response

    goal_state_valid, goal_state = _getInverseKinematicsSolution(start_state, goal_pose)

    if not goal_state_valid:
        response = moveitPlanFromPoseToPoseSrvResponse()
        response.plan = RobotTrajectory()
        response.success = Bool(False)

        return response

    joint_states_at_goal = list(goal_state.solution.joint_state.position)
    joint_values_at_goal = copy.deepcopy(joint_states_at_goal[:7])
    robot.set_start_state(start_state)
    robot.set_joint_value_target(joint_values_at_goal)
    trajectory = robot.plan()

    response = moveitPlanFromPoseToPoseSrvResponse()
    response.plan = trajectory
    response.success = Bool(True)

    return responses

def _getInverseKinematicsSolution(initial_state,
                                goal_pose):
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


def planToPose(req):

    start_state = _getCurrentState()
    goal_pose = req.pose

    goal_state_valid, goal_state = _getInverseKinematicsSolution(start_state, goal_pose)

    if not goal_state_valid:
        response = moveitPlanToPoseSrvResponse()
        response.plan = RobotTrajectory()
        response.success = Bool(False)

        return response

    joint_states_at_goal = list(goal_state.solution.joint_state.position)
    joint_values_at_goal = copy.deepcopy(joint_states_at_goal[:7])

    robot.set_start_state(start_state)
    robot.set_joint_value_target(joint_values_at_goal)
    trajectory = robot.plan()

    response = moveitPlanToPoseSrvResponse()
    response.plan = trajectory
    response.success = Bool(True)

    return response

def _moveToNamed(name):
    """ moves robot to predefined named pose
    """

    if not len(name):
        print("ERROR: Specify a named pose")
        return False

    robot.set_named_target(name)
    plan = robot.go(wait=True)
    robot.stop()
    robot.clear_pose_targets()

    return True

def _closeGripper():
    """ you know what this does, come on!
    """

    gripper.set_named_target("closed")
    plan = gripper.go(wait=True)
    gripper.stop()
    gripper.clear_pose_targets()

    rospy.sleep(1)

    return True

def closeGripper(req):

    _closeGripper()
    resp = moveitGripperCloseSrvResponse()
    resp.success.data = True

    return resp

def _openGripper():
    """ you know what this does, come on!
    """

    gripper.set_named_target("open")
    plan = gripper.go(wait=True)
    gripper.stop()
    gripper.clear_pose_targets()

    rospy.sleep(1)

    return True

def openGripper(req):

    _openGripper()
    resp = moveitGripperOpenSrvResponse()
    resp.success.data = True

    return resp

def moveToNamed(req):

    _moveToNamed(req.name.data)

    resp = moveitMoveToNamedSrvResponse()
    resp.success.data = True

    return resp

def execute(req):

    robot.execute(req.trajectory, wait=True)
    robot.stop()
    robot.clear_pose_targets()

    resp = moveitExecuteSrvResponse()
    resp.success.data = True

    return resp

def _getCurrentState():

    return robot.get_current_state()

def getCurrentState(req):

    return _getCurrentState()

def getJointPositionAtNamed(req):
    target_values = move_group.get_named_target_values(req.target.data)
    resp = moveitGetJointPositionAtNamedResponse()
    resp.joint_position.data.append(target_values["panda_joint1"])
    resp.joint_position.data.append(target_values["panda_joint2"])
    resp.joint_position.data.append(target_values["panda_joint3"])
    resp.joint_position.data.append(target_values["panda_joint4"])
    resp.joint_position.data.append(target_values["panda_joint5"])
    resp.joint_position.data.append(target_values["panda_joint6"])
    resp.joint_position.data.append(target_values["panda_joint7"])
    return resp

def setMaxAccelerationScalingFactor(req):

    robot.set_max_acceleration_scaling_factor(req.value.data)

    resp = moveitSetMaxAccSrvResponse()
    resp.success.data = True

    return resp

def setMaxVelocityScalingFactor(req):
    robot.set_max_velocity_scaling_factor(req.value.data)

    resp = moveitSetMaxVelSrvResponse()
    resp.success.data = True

    return resp

def setPlanningTime(req):
    robot.set_planning_time(req.value.data)

    resp = moveitSetPlanningTimeSrvResponse()
    resp.success.data = True

    return resp

def setNumPlanningAttempts(req):
    robot.set_num_planning_attempts(req.value.data)

    resp = moveitNumPlanningAttemptsSrvResponse()
    resp.success.data = True

    return resp

if __name__ == '__main__':

    baseServiceName = "/fh_handover/moveit/"

    rospy.init_node('moveit_service_node', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    r1 = moveit_commander.RobotCommander()
    robot = moveit_commander.MoveGroupCommander("panda_arm")

    gripper = moveit_commander.MoveGroupCommander("panda_hand")

    moveToNameService = rospy.Service(baseServiceName + "move_to_named", moveitMoveToNamedSrv, moveToNamed)
    gripperCloseService = rospy.Service(baseServiceName + "gripper_close", moveitGripperCloseSrv, closeGripper)
    gripperOpenService = rospy.Service(baseServiceName + "gripper_open", moveitGripperOpenSrv, openGripper)
    planToNameService = rospy.Service(baseServiceName + "plan_to_named", moveitPlanToNamedSrv, planToNamed)
    planToPoseService = rospy.Service(baseServiceName + "plan_to_pose", moveitPlanToPoseSrv, planToPose)
    planFromPoseToPoseService = rospy.Service(baseServiceName + "plan_from_pose_to_pose", moveitPlanFromPoseToPoseSrv, planFromPoseToPose)
    moveToPoseService = rospy.Service(baseServiceName + "move_to_pose", moveitMoveToPoseSrv, moveToPose)
    executeService = rospy.Service(baseServiceName + "execute", moveitExecuteSrv, execute)
    robotStateService = rospy.Service(baseServiceName + "getRobotState", moveitRobotStateSrv, getCurrentState)
    getJointPositionAtNamedService = rospy.Service(baseServiceName + "getJointPositionAtNamed", moveitGetJointPositionAtNamed, getJointPositionAtNamed)
    robotMaxAccService = rospy.Service(baseServiceName + "set_max_acceleration_scaling_factor", moveitSetMaxAccSrv, setMaxAccelerationScalingFactor)
    robotMaxVelService = rospy.Service(baseServiceName + "set_max_velocity_scaling_factor", moveitSetMaxVelSrv, setMaxVelocityScalingFactor)
    planningTimeService = rospy.Service(baseServiceName + "set_planning_time", moveitSetPlanningTimeSrv, setPlanningTime)
    setNumPlanningAttemptsService = rospy.Service(baseServiceName + "set_num_planning_attempts", moveitNumPlanningAttemptsSrv, setNumPlanningAttempts)


    robot.set_max_acceleration_scaling_factor(0.1)
    robot.set_max_velocity_scaling_factor(0.1)
    robot.set_planning_time(0.1)
    robot.set_num_planning_attempts(25)

    rospy.spin()
