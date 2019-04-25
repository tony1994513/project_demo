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

SIMULATION = False
HUMAN_CONTROL = True

picking_pose = _Constant.memoeryChip_pickPose
insert_pose = _Constant.memoeryChip_insertPose
prepick_offset = _Constant.prepick_offset
preinsert_offset = _Constant.preinsert_offset

class MoveToReadyPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        if not SIMULATION:
            arm_utils.gripper_control("open")
        arm_utils.speed_set(group, 1)
        home_position = _Constant.home 
        home_plan = arm_utils.fK_calculate(group,home_position)
        arm_utils.execute_plan(group,home_plan)

        return "succuss"

class MoveToMiddlePose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        if not SIMULATION:
            arm_utils.gripper_control("open")
        arm_utils.speed_set(group, 1)
        home_position = _Constant.home 
        home_plan = arm_utils.fK_calculate(group,home_position)
        arm_utils.execute_plan(group,home_plan)

        return "succuss"

class MoveToCameraDetectionPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        # arm_utils.speed_set(group, 0.6)
        plan = arm_utils.fK_calculate(group,_Constant.memoeryChip_camera_detection)
        arm_utils.execute_plan(group,plan)
        rospy.sleep(3)
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
            rospy.loginfo("Picking pose set by human")
            picking_pose = picking_pose

        return "succuss"

class MoveToPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
        arm_utils.speed_set(group, 0.6)
        prepick = copy.deepcopy(picking_pose)
        prepick.position.z += prepick_offset
        plan = arm_utils.iK_calculate(group,prepick)
        arm_utils.execute_plan(group,plan)
        return "succuss"


class MoveToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
        arm_utils.speed_set(group, 0.5)
        pick = copy.deepcopy(picking_pose)
        plan = arm_utils.iK_calculate(group,pick)
        new_traj = arm_utils.linear_interplotation(pick)
        (plan, fraction) = group.compute_cartesian_path(
                            new_traj,   # waypoints to follow
                            0.001,        # eef_step
                            0.0)         # jump_threshold
        arm_utils.execute_plan(group,plan)
            
        if not SIMULATION:
            rospy.sleep(2)
            arm_utils.gripper_control("chip_close")
            rospy.sleep(1)
            # rospy.sleep(2)
            # arm_utils.gripper_control("open")
        return "succuss"

class BackToPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
        arm_utils.speed_set(group, 1)
        prepick = copy.deepcopy(picking_pose)
        prepick.position.z += prepick_offset
        new_traj = arm_utils.linear_interplotation_back(prepick)
        (plan, fraction) = group.compute_cartesian_path(
                            new_traj,   # waypoints to follow
                            0.1,        # eef_step
                            0.0)         # jump_threshold
        arm_utils.execute_plan(group,plan)
        return "succuss"

class MoveToPreInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,insert_pose
        arm_utils.speed_set(group, 1)
        preinsert = copy.deepcopy(insert_pose)
        preinsert.position.z += preinsert_offset
        plan = arm_utils.iK_calculate(group,preinsert)
        arm_utils.execute_plan(group,plan)
        return "succuss"

class MoveToInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,insert_pose
        arm_utils.speed_set(group, 0.5)
        insert_pose = copy.deepcopy(insert_pose)
        new_traj = arm_utils.linear_interplotation(insert_pose)
        (plan, fraction) = group.compute_cartesian_path(
                            new_traj,   # waypoints to follow
                            0.001,        # eef_step
                            0.0)         # jump_threshold
        arm_utils.execute_plan(group,plan)
        if not SIMULATION:
            rospy.sleep(2)
            arm_utils.gripper_control("open")
            rospy.sleep(1)
        return "succuss"

class BackToPreInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        arm_utils.speed_set(group, 1)
        preinsert = copy.deepcopy(insert_pose)
        preinsert.position.z += preinsert_offset
        new_traj = arm_utils.linear_interplotation_back(preinsert)
        (plan, fraction) = group.compute_cartesian_path(
                            new_traj,   # waypoints to follow
                            0.1,        # eef_step
                            0.0)         # jump_threshold
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
    group.set_max_velocity_scaling_factor(1)
    group.set_max_acceleration_scaling_factor(1)
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
                               transitions={'succuss':BackToPrePickPosition.__name__}), 

        smach.StateMachine.add(BackToPrePickPosition.__name__, BackToPrePickPosition(), 
                               transitions={'succuss':MoveToPreInsertPosition.__name__}),                                                 

        smach.StateMachine.add(MoveToPreInsertPosition.__name__, MoveToPreInsertPosition(), 
                               transitions={'succuss':MoveToInsertPosition.__name__}), 

        smach.StateMachine.add(MoveToInsertPosition.__name__, MoveToInsertPosition(), 
                               transitions={'succuss':BackToPreInsertPosition.__name__}), 

        smach.StateMachine.add(BackToPreInsertPosition.__name__, BackToPreInsertPosition(), 
                               transitions={'succuss':"Done"}),           

    outcome = sm.execute()
    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
