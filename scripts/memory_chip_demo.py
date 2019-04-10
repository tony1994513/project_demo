#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String,Bool
from moveit_api_warpper import arm_utils


DEBUG = True
anomaly_flag = None

def detection_callback(data):
    global anomaly_flag
    anomaly_flag = data.data
    # rospy.loginfo(data)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('memory_chip_demo',anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("tx90_arm")
    # set moveit param
    robot.set_planning_time(15)    
    robot.allow_replanning(True)
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)
    group.set_goal_joint_tolerance(0.001)
    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.001)

    rospy.wait_for_message('anomaly_detection_pub',Bool,timeout=3)
    rospy.Subscriber("anomaly_detection_pub", Bool, detection_callback)
    rate = rospy.Rate(20)
    rospy.sleep(1)

    if DEBUG:
        print "============ Reference frame for this robot: %s" % group.get_planning_frame()
        print "============ End-effector link for this group: %s" % group.get_end_effector_link()
        print "============ Robot Groups:"
        print robot.get_group_names()
        print "============ Printing robot state"
        print arm_utils.print_robot_Jointstate(robot)
        print "============"

    home_position = [-0.14257286131927796, 1.0905119122949902, -0.59253489336999264, -3.1406088825859366, 0.6986560818776872, -2.999702977027948]
    detection_position = [-0.14257286131927796, 1.0905119122949902, -0.59253489336999264, -3.1406088825859366, 0.6986560818776872, -2.999702977027948]
    detection_position = [-0.14257286131927796, 1.0905119122949902, -0.59253489336999264, -3.1406088825859366, 0.6986560818776872, -2.999702977027948]
    chip_above_position = [-0.14257286131927796, 1.0905119122949902, -0.59253489336999264, -3.1406088825859366, 0.6986560818776872, -2.999702977027948]

    while not rospy.is_shutdown():
        if anomaly_flag=False:
            home_plan = arm_utils.fK_calculate(group,home_position)
            arm_utils.execute_plan(group,home_plan)
            arm_utils.gripper_control("open")

            plan = arm_utils.fK_calculate(group,detection_position)
            arm_utils.execute_plan(group,plan)
            arm_utils.gripper_control("open")
            
            plan = arm_utils.fK_calculate(group,detection_position)
            arm_utils.execute_plan(group,plan)

            plan = arm_utils.fK_calculate(group,detection_position)
            arm_utils.execute_plan(group,plan)

            print arm_utils.print_robot_Jointstate(robot)

            target_joint = [-0.14257286131927796, 1.0905119122949902, -0.59253489336999264, -3.1406088825859366, 0.6986560818776872, -2.999702977027948]
            plan = arm_utils.fK_calculate(group,target_joint)
            arm_utils.execute_plan(group,plan)
        elif anomaly_flag=True:
            group.stop()


 rate.sleep()

# Shut down MoveIt cleanly
moveit_commander.roscpp_shutdown()
# Exit MoveIt
moveit_commander.os._exit(0)

if __name__ == '__main__':
    sys.exit(main()) 