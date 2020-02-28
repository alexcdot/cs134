#!/usr/bin/env python

import sys
import rospy
import time

from boogaloo.msg import JointCommand, PoseCommand, RobotState, MassChange

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

    cmd.pos.z = 0.17
    cmd.pos.x = 0.65
    cmd.pos.y = 0.22
    pub_and_wait(tip_pub, cmd)

    cmd.gripper = 1
    pub_and_wait(tip_pub, cmd)

    mass.mass_status = MassChange.BOTTLE_TOP
    mass_pub.publish(mass)

    cmd = JointCommand()
    cmd.gripper = 1
    cmd.pose_follow = True

    pub_and_wait(joint_pub, cmd)

    
    print('yay!')
    
    

    