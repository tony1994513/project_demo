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

robot = None
group = None
object_pose = None
insert_position = []

SIMULATION = True

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


class MoveToChipPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        for position in _Constant.pick_position_list:
            plan = arm_utils.fK_calculate(group,position)
            arm_utils.execute_plan(group,plan)
        if not SIMULATION:
            arm_utils.gripper_control("chip_close")
        return "succuss"

class MoveToChipPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        for position in _Constant.prepick_position_list:
            plan = arm_utils.fK_calculate(group,position)
            arm_utils.execute_plan(group,plan)
        if not SIMULATION:
            arm_utils.gripper_control("chip_close")
        return "succuss"


class MoveToInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        for position in _Constant.insert_position_list:
            plan = arm_utils.fK_calculate(group,position)
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
    rospy.init_node('memory_chip_demo',anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("tx90_arm")
    group.set_max_velocity_scaling_factor(0.01)
    group.set_max_acceleration_scaling_factor(0.01)
    group.set_goal_joint_tolerance(0.001)
    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.001)

    sm = smach.StateMachine(outcomes=['Done'])
    with sm:
        smach.StateMachine.add(MoveToReadyPose.__name__, MoveToReadyPose(), 
                               transitions={'succuss':MoveToChipPickPosition.__name__})     

        smach.StateMachine.add(MoveToChipPickPosition.__name__, MoveToChipPickPosition(), 
                               transitions={'succuss':MoveToChipPrePickPosition.__name__})    

        smach.StateMachine.add(MoveToChipPrePickPosition.__name__, MoveToChipPrePickPosition(), 
                               transitions={'succuss':MoveToInsertPosition.__name__}),      

        smach.StateMachine.add(MoveToInsertPosition.__name__, MoveToInsertPosition(), 
                               transitions={'succuss':MoveToPreInsertPosition.__name__}),   
 
        smach.StateMachine.add(MoveToPreInsertPosition.__name__, MoveToPreInsertPosition(), 
                               transitions={'succuss':"Done"}),                                                                                                

    outcome = sm.execute()


if __name__ == '__main__':
    sys.exit(main())