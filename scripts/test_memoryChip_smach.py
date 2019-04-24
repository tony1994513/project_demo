#!/usr/bin/env python
import rospy
import os,sys
import numpy as np
import smach
import smach_ros
from std_msgs.msg import String,Bool
from moveit_api_warpper import arm_utils
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from linemod_pose_estimation.srv import linemod_pose, linemod_poseRequest, linemod_poseResponse
from staubli_val3_driver.srv import IOCommand,IOCommandRequest,IOCommandResponse
from moveit_api_warpper import _Constant
import copy

robot = None
group = None
picking_pose = None

insert_position = []

SIMULATION = True
HUMAN_CONTROL = False

class MoveToReadyPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        home_position = _Constant.home 
        home_plan = arm_utils.fK_calculate(group,home_position)
        arm_utils.execute_plan(group,home_plan)
        if not SIMULATION:
            arm_utils.gripper_control("open")
        return "succuss"

class MoveToMiddlePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        home_position = _Constant.home 
        home_plan = arm_utils.fK_calculate(group,home_position)
        arm_utils.execute_plan(group,home_plan)
        if not SIMULATION:
            arm_utils.gripper_control("open")
        return "succuss"

class MoveToCameraDetectionPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group

        plan = arm_utils.fK_calculate(group,_Constant.camera_detection_position)
        arm_utils.execute_plan(group,plan)
            
        if not SIMULATION:
            arm_utils.gripper_control("open")
        return "succuss"

class DetermineObjectPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
        if not HUMAN_CONTROL:
            rospy.sleep(1)
            object_pose = arm_utils.object_pose(0)
            print "object pose"
            print [object_pose.pose.translation.x, object_pose.pose.translation.y, object_pose.pose.translation.z,
            object_pose.pose.rotation.x ,object_pose.pose.rotation.y,object_pose.pose.rotation.z,object_pose.pose.rotation.w] 
            picking_pose = arm_utils.objectPoseToPickpose(object_pose)
            print "raw picking pose"
            print [picking_pose.position.x,picking_pose.position.y,picking_pose.position.z,picking_pose.orientation.x
                ,picking_pose.orientation.y,picking_pose.orientation.z,picking_pose.orientation.w]
        else:
            rospy.loginfo("jointAngle mode")
            fk_compute_service
        return "succuss"

class MoveToPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose

        prepick = copy.deepcopy(picking_pose)
        prepick.position.z += 0.17
        plan = arm_utils.iK_calculate(group,prepick)
        arm_utils.execute_plan(group,plan)
        return "succuss"


class MoveToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
        if HUMAN_CONTROL:
            for position in _Constant.pick_position_list:
                plan = arm_utils.fK_calculate(group,position)
                arm_utils.execute_plan(group,plan)
        else:
            pick = copy.deepcopy(picking_pose)
            plan = arm_utils.iK_calculate(group,pick)
            new_traj = arm_utils.linear_interplotation(pick)
            (plan, fraction) = group.compute_cartesian_path(
                             new_traj,   # waypoints to follow
                             0.001,        # eef_step
                             0.0)         # jump_threshold
            arm_utils.execute_plan(group,plan)
            
        if not SIMULATION:
            arm_utils.gripper_control("chip_close")
        return "succuss"

class BackToPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
        if HUMAN_CONTROL:
            for position in _Constant.pick_position_list:
                plan = arm_utils.fK_calculate(group,position)
                arm_utils.execute_plan(group,plan)
        else:
            prepick = copy.deepcopy(picking_pose)
            plan = arm_utils.iK_calculate(group,prepick)
            arm_utils.execute_plan(group,plan)
            
        if not SIMULATION:
            arm_utils.gripper_control("chip_close")
        return "succuss"

class MoveToPreInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        plan = arm_utils.fK_calculate(group,home_position)
        arm_utils.execute_plan(group,plan)
        if not SIMULATION:
            arm_utils.gripper_control("open")
        return "succuss"

class MoveToPreInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        for position in _Constant.preinsert_position_list:
            plan = arm_utils.fK_calculate(group,position)
            arm_utils.execute_plan(group,plan)
        if not SIMULATION:
            arm_utils.gripper_control("open")
        return "succuss"


def main():
    global robot,group
    rospy.init_node("memory_chip_demo",anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("tx90_arm")
    # group.set_max_velocity_scaling_factor(0.01)
    # group.set_max_acceleration_scaling_factor(0.01)
    group.set_goal_joint_tolerance(0.001)
    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.001)

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:
        smach.StateMachine.add(MoveToReadyPose.__name__, MoveToReadyPose(), 
                               transitions={'succuss':MoveToCameraDetectionPosition.__name__})     

        smach.StateMachine.add(MoveToCameraDetectionPosition.__name__, MoveToCameraDetectionPosition(), 
                               transitions={'succuss':DetermineObjectPose.__name__})    

        smach.StateMachine.add(DetermineObjectPose.__name__, DetermineObjectPose(), 
                               transitions={'succuss':MoveToMiddlePose.__name__}),                                                                                                

        smach.StateMachine.add(MoveToMiddlePose.__name__, MoveToMiddlePose(), 
                               transitions={'succuss':MoveToPrePickPosition.__name__})  

        smach.StateMachine.add(MoveToPrePickPosition.__name__, MoveToPrePickPosition(), 
                               transitions={'succuss':MoveToPickPosition.__name__}),   

        smach.StateMachine.add(MoveToPickPosition.__name__, MoveToPickPosition(), 
                               transitions={'succuss':"Done"}), 
                                                 
    outcome = sm.execute()
    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
