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
        home_position = _Constant.home 
        home_plan = arm_utils.fK_calculate(group,home_position)
        arm_utils.execute_plan(group,home_plan)

        return "succuss"

class MoveToCameraDetectionPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        plan = arm_utils.fK_calculate(group,_Constant.memoeryChip_camera_detection)
        arm_utils.execute_plan(group,plan)
        rospy.sleep(1)
        return "succuss"

class DetermineObjectPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
<<<<<<< HEAD

            # print "object pose"
            # print [object_pose.pose.translation.x, object_pose.pose.translation.y, object_pose.pose.translation.z,
            # object_pose.pose.rotation.x ,object_pose.pose.rotation.y,object_pose.pose.rotation.z,object_pose.pose.rotation.w] 
            # picking_pose = arm_utils.objectPoseToPickpose(object_pose)
            # print "raw picking pose"
            # print [picking_pose.position.x,picking_pose.position.y,picking_pose.position.z,picking_pose.orientation.x
            #     ,picking_pose.orientation.y,picking_pose.orientation.z,picking_pose.orientation.w]
        # rospy.sleep(1)
        # if not SIMULATION:
            # rospy.sleep(1)
            # object_pose = arm_utils.object_pose(0)
            # rospy.sleep(1)
        # rospy.loginfo("Picking pose set by human")
=======
        if not SIMULATION:
            rospy.sleep(1)
            object_pose = arm_utils.object_pose(0)
            print object_pose
        rospy.loginfo("Picking pose set by human")
>>>>>>> cb6b254ec26f5d8d08841d3407d2ce4c9012ca0f
        picking_pose = picking_pose

        return "succuss"

class MoveToPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,picking_pose
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
        pick = _Constant.memoeryChip_pickJoint_1
        plan = arm_utils.fK_calculate(group,pick)
        arm_utils.execute_plan(group,plan)

        pick = _Constant.memoeryChip_pickJoint_2
        plan = arm_utils.fK_calculate(group,pick)
        arm_utils.execute_plan(group,plan)
            
        if not SIMULATION:
            rospy.sleep(1)
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
        prepick = copy.deepcopy(picking_pose)
        prepick.position.z += prepick_offset
        new_traj = arm_utils.linear_interplotation_back(prepick)
        (plan, fraction) = group.compute_cartesian_path(
                            new_traj,   # waypoints to follow
                            0.1,        # eef_step
                            0.0)         # jump_threshold
        arm_utils.execute_plan(group,plan)
        return "succuss"


class MoveToAnomalyPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
<<<<<<< HEAD
        global robot,group,insert_pose

        Anomalyprepick = copy.deepcopy(insert_pose)
        Anomalyprepick.position.z += 0.17
=======
        global robot,group
        rospy.sleep(2)
        Anomalyprepick = copy.deepcopy(_Constant.memoeryChip_insertPose)
        Anomalyprepick.position.y += 0.05
        Anomalyprepick.position.z += 0.1
>>>>>>> cb6b254ec26f5d8d08841d3407d2ce4c9012ca0f
        plan = arm_utils.iK_calculate(group,Anomalyprepick)
        arm_utils.execute_plan(group,plan)
        return "succuss"

class MoveToAnomalyPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
<<<<<<< HEAD
        global robot,group,insert_pose
        # Anomalypick = copy.deepcopy(insert_pose)
        # Anomalypick.position.y += 0.002
        # Anomalypick.position.z += 0.013
        # new_traj = arm_utils.linear_interplotation(Anomalypick)
        # (plan, fraction) = group.compute_cartesian_path(
        #                     new_traj,   # waypoints to follow
        #                     0.001,        # eef_step
        #                     0.0)         # jump_threshold
        Anomalypick = _Constant.anomalyJoint
        plan = arm_utils.fK_calculate(group,Anomalypick)
        arm_utils.execute_plan(group,plan)
        rospy.sleep(2)
=======
        global robot,group
        Anomalypick = copy.deepcopy(_Constant.memoeryChip_insertPose)
        Anomalypick.position.y += 0.005
        new_traj = arm_utils.linear_interplotation(Anomalypick)
        (plan, fraction) = group.compute_cartesian_path(
                            new_traj,   # waypoints to follow
                            0.001,        # eef_step
                            0.0)         # jump_threshold
        arm_utils.execute_plan(group,plan)
        rospy.sleep(3)
>>>>>>> cb6b254ec26f5d8d08841d3407d2ce4c9012ca0f
        return "succuss"

class BackToAnomalyPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
<<<<<<< HEAD
        global robot,group,insert_pose
        Anomalyprepick = copy.deepcopy(insert_pose)
        Anomalyprepick.position.z += 0.08
        plan = arm_utils.iK_calculate(group,Anomalyprepick)
=======
        global robot,group
        Anomalyprepick = copy.deepcopy(_Constant.memoeryChip_insertPose)
        Anomalyprepick.position.z += 0.05
        new_traj = arm_utils.linear_interplotation_back(Anomalyprepick)
        (plan, fraction) = group.compute_cartesian_path(
                            new_traj,   # waypoints to follow
                            0.1,        # eef_step
                            0.0)         # jump_threshold
>>>>>>> cb6b254ec26f5d8d08841d3407d2ce4c9012ca0f
        arm_utils.execute_plan(group,plan)
        return "succuss"

class MoveToPreInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,insert_pose
        preinsert = _Constant.preNomalJoint
        plan = arm_utils.fK_calculate(group,preinsert)
        arm_utils.execute_plan(group,plan)
        rospy.sleep(2)
        return "succuss"

class MoveToInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group,insert_pose
        insert = _Constant.nomalJoint
        plan = arm_utils.fK_calculate(group,insert)
        arm_utils.execute_plan(group,plan)
        if not SIMULATION:
            rospy.sleep(1)
            arm_utils.gripper_control("open")
            rospy.sleep(1)
        return "succuss"

class BackToPreInsertPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        global robot,group
        preinsert = copy.deepcopy(insert_pose)
        preinsert.position.z += 0.1
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
                               transitions={'succuss':MoveToAnomalyPrePickPosition.__name__}),                                                 

        smach.StateMachine.add(MoveToAnomalyPrePickPosition.__name__, MoveToAnomalyPrePickPosition(), 
                               transitions={'succuss':MoveToAnomalyPickPosition.__name__}),    

        smach.StateMachine.add(MoveToAnomalyPickPosition.__name__, MoveToAnomalyPickPosition(), 
                               transitions={'succuss':BackToAnomalyPrePickPosition.__name__}),    

        smach.StateMachine.add(BackToAnomalyPrePickPosition.__name__, BackToAnomalyPrePickPosition(), 
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
