#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy as ros
import time
from threading import Lock
from copy import deepcopy

from boogaloo.msg import JointCommand, PoseCommand, ThrowCommand, RobotState, MassChange, Detection

active_state = None
multi_step = 0

last_msg_time = None

curr_state = None
bottle_cap_det = None

joint_pub = None
tip_pub = None
throw_pub = None
mass_pub = None

lock = Lock()

def publish_msg(pub, msg):
    global last_msg_time
    pub.publish(msg)
    last_msg_time = ros.Time.now()

# Callback to update robot current state
def robot_state_callback(data):
    global curr_state

    with lock:
        curr_state = data

# Callback to get new bottle cap dets. Only saves when waiting for a bottle det
def bottle_cap_det_callback(data):
    global bottle_cap_det, active_state

    with lock:
        if active_state == wait_bottle_cap_det:
            bottle_cap_det = data

def startup():
    global joint_pub, mass_pub, active_state
    joint_msg = JointCommand()
    mass_msg = MassChange()
    publish_msg(joint_pub, joint_msg)
    publish_msg(mass_pub, mass_msg)
    active_state = wait_bottle_cap_det

def wait_bottle_cap_det():
    global bottle_cap_det, active_state
    print('Waiting for det')
    if bottle_cap_det is not None:
        active_state = grab_bottle_cap

def grab_bottle_cap():
    global bottle_cap_det, tip_pub, mass_pub, multi_step, active_state
    pose_msg = PoseCommand()
    pose_msg.pose_follow = True

    mass_msg = MassChange()

    print('Grabbing stage', multi_step)

    if multi_step == 0:
        pose_msg.pos = deepcopy(bottle_cap_det.position)
        pose_msg.pos.z += 0.05
        mass_msg.mass_status = mass_msg.EMPTY
        multi_step += 1

    elif multi_step == 1:
        pose_msg.pos = deepcopy(bottle_cap_det.position)
        mass_msg.mass_status = mass_msg.EMPTY
        multi_step += 1

    elif multi_step == 2:
        pose_msg.pos = deepcopy(bottle_cap_det.position)
        pose_msg.gripper = 1
        mass_msg.mass_status = mass_msg.EMPTY
        multi_step += 1

    elif multi_step == 3:
        pose_msg.pos = deepcopy(bottle_cap_det.position)
        pose_msg.gripper = 1
        mass_msg.mass_status = mass_msg.BOTTLE_TOP
        multi_step = 0
        active_state = return_home

    print(pose_msg)
    publish_msg(tip_pub, pose_msg)
    publish_msg(mass_pub, mass_msg)

def return_home():
    global joint_pub, active_state
    print('Returning home')
    joint_msg = JointCommand()
    joint_msg.gripper = 1
    publish_msg(joint_pub, joint_msg)

# Main logic loop
def main_loop(timer_event):
    global active_state, curr_state, last_msg_time

    with lock:
        # Make sure there's enough time to enterpret message before expecting changes
        if (ros.Time.now() - last_msg_time).to_sec() < 0.1:
            return

        # Logic to choose next actions
        if curr_state.is_at_target:
            active_state()
        # Logic to potentially interupt current action
        else:
            pass

# Initialize and first time setup
def main():
    global joint_pub, tip_pub, throw_pub, mass_pub, curr_state, active_state, last_msg_time

    ros.init_node('logic_controller')

    # Init subscribers
    ros.Subscriber("/robot_state", RobotState, robot_state_callback)
    ros.Subscriber("/bottle_cap_dets", Detection, bottle_cap_det_callback)

    # Init publishers
    joint_pub = ros.Publisher('/joint_goal', JointCommand, queue_size=10)
    tip_pub = ros.Publisher('/tip_goal', PoseCommand, queue_size=10)
    throw_pub = ros.Publisher('/throw_goal', ThrowCommand, queue_size=10)
    mass_pub = ros.Publisher('/mass_updates', MassChange, queue_size=10)

    # Make sure we have an accurate robot state
    ros.sleep(ros.Duration(2.0))
    ros.loginfo('Waiting for current state...')
    while curr_state is not None and not ros.is_shutdown:
        ros.sleep(ros.Duration(0.1))
    ros.loginfo('Got current state!')

    active_state = startup

    last_msg_time = ros.Time.now()

    # Setup main loop and start running
    ros.Timer(ros.Duration(0.01), main_loop)
    ros.spin()

if __name__ == '__main__':
    main()