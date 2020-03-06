#!/usr/bin/env python

import sys
import rospy
import time

from boogaloo.msg import JointCommand, PoseCommand, RobotState, MassChange, ThrowCommand

curr_state = RobotState()

def robot_state_callback(data):
    global curr_state
    curr_state = data

def pub_and_wait(pub, msg):
    global curr_state

    pub.publish(msg)

    rospy.sleep(rospy.Duration(0.5))

    while curr_state.is_at_target == False and not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(0.1))

if __name__ == '__main__':
    rospy.init_node('scripted_runner')

    rospy.Subscriber("/robot_state", RobotState, robot_state_callback)
    joint_pub = rospy.Publisher('/joint_goal', JointCommand, queue_size=10)
    tip_pub = rospy.Publisher('/tip_goal', PoseCommand, queue_size=10)
    throw_pub = rospy.Publisher('/throw_goal', ThrowCommand, queue_size=10)
    mass_pub = rospy.Publisher('/mass_updates', MassChange, queue_size=10)

    rospy.sleep(rospy.Duration(2))

    while curr_state.is_at_target == False and not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(0.1))

    mass = MassChange()
    mass.mass_status = MassChange.EMPTY
    mass_pub.publish(mass)

    cmd = JointCommand()

    pub_and_wait(joint_pub, cmd)

    cmd = PoseCommand()
    cmd.pos.x = 0.63
    cmd.pos.y = 0.20
    cmd.pos.z = 0.2
    cmd.wrist_angle = 0
    cmd.wrist_roll = -2.4
    cmd.gripper = 0
    cmd.pose_follow = True

    pub_and_wait(tip_pub, cmd)

    cmd.pos.z = -0.02
    pub_and_wait(tip_pub, cmd)

    cmd.gripper = 1

    pub_and_wait(tip_pub, cmd)

    mass.mass_status = MassChange.BOTTLE_SIDE
    mass_pub.publish(mass)

    cmd.pos.z = 0.2

    pub_and_wait(tip_pub, cmd)

    cmd.wrist_roll = 0
    cmd.wrist_angle = 1.57

    pub_and_wait(tip_pub, cmd)

    cmd.pos.z = 0.05
    pub_and_wait(tip_pub, cmd)

    mass.mass_status = MassChange.EMPTY
    mass_pub.publish(mass)

    cmd.gripper = 0
    pub_and_wait(tip_pub, cmd)

    cmd.pos.x = 0.55
    cmd.pos.z = 0.20
    cmd.wrist_angle = 0
    pub_and_wait(tip_pub, cmd)

    cmd.pos.z = 0.18
    cmd.pos.x = 0.65
    cmd.pos.y = 0.21
    pub_and_wait(tip_pub, cmd)

    cmd.gripper = 1
    pub_and_wait(tip_pub, cmd)

    mass.mass_status = MassChange.BOTTLE_TOP
    mass_pub.publish(mass)

    cmd = JointCommand()
    cmd.gripper = 1
    cmd.pose_follow = True

    pub_and_wait(joint_pub, cmd)

    cmd = ThrowCommand()
    cmd.yaw_ang = 0.0
    cmd.elbow_ang = 1.57
    cmd.wrist_ang = 1.7
    cmd.elbow_vel = 4.0
    cmd.wrist_vel = 6.0
    cmd.elbow_rest = 1.57
    cmd.wrist_rest = 1.5

    pub_and_wait(throw_pub, cmd)

    cmd = JointCommand()
    # cmd.gripper = 1
    cmd.pose_follow = True

    pub_and_wait(joint_pub, cmd)

    
    print('yay!')
    
    
# GOOD THROWS 
    # (87 cm)
    # cmd.elbow_ang = 1.57
    # cmd.wrist_ang = 1.7
    # cmd.elbow_vel = 4.0
    # cmd.wrist_vel = 6.0
    # cmd.elbow_rest = 1.57
    # cmd.wrist_rest = 1.5

    # (67.5 cm)
    # cmd.elbow_ang = 1.7
    # cmd.wrist_ang = 1.8
    # cmd.elbow_vel = 4.0
    # cmd.wrist_vel = 6.0
    # cmd.elbow_rest = 1.7
    # cmd.wrist_rest = 1.9

