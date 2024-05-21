#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import numpy as np
from math import pi
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_from_euler


rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "rldp5_arm"
arm_group = moveit_commander.MoveGroupCommander(group_name)

rospy.sleep(1) 

def go_to_joint_state(goal):
    
    print("Current joint values:", arm_group.get_current_joint_values())

    joint_goal = JointState()
    joint_goal.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
    joint_goal.position = goal
    
    input("======== Press enter to move")
    arm_group.go(joint_goal, wait=True)
    arm_group.stop()
    print("Moved to pose\n")
    
    arm_group.clear_pose_targets()


def listener():

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    deg = pi/180
    

    go_to_joint_state([-20*deg, -45*deg, -35*deg, -100*deg, 0])
    go_to_joint_state([-20*deg, 35*deg, -115*deg, -79*deg, 0])

    go_to_joint_state([0, 50*deg, -115*deg, -78*deg, 0])
    go_to_joint_state([0, -45*deg, -35*deg, -100*deg, 0])

    go_to_joint_state([20*deg, -45*deg, -35*deg, -100*deg, 0])
    go_to_joint_state([20*deg, 35*deg, -115*deg, -79*deg, 0])

    go_to_joint_state([0, 0, -62*deg, -100*deg, 0])
    #go_to_joint_state([-0.71, -0.41, -0.97, -1.6, 0.71])
    #go_to_joint_state([0.71, -0.41, -0.97, -1.6, -0.71])
    

    rospy.spin()

if __name__ == '__main__':
    listener()