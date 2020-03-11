#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy as ros
import time
import numpy as np
import math
import random
from threading import Lock
from copy import deepcopy

from boogaloo.msg import JointCommand, PoseCommand, ThrowCommand, RobotState, MassChange, Detection
from geometry_msgs.msg import Vector3

# Function to call
active_state = None

# Counter for multi-step states (to consolidate states that always happen senquentially)
multi_step = 0

# Last time we sent a message - used for rate limiting
last_msg_time = None

# Current robot state
curr_state = None

# Detection type names
CAP_DET = 'CAP_DET'
RIM_DET = 'RIM_DET'
BAND_DET = 'BAND_DET'
# Most recent detections
detections = {
    CAP_DET: None,
    RIM_DET: None,
    BAND_DET: None
}
det_history = {
    CAP_DET: [],
    RIM_DET: [],
    BAND_DET: []
}
# Whether or not we want new dets
want_detections = {
    CAP_DET: False,
    RIM_DET: False,
    BAND_DET: False
}

# Publishers
joint_pub = None
tip_pub = None
throw_pub = None
mass_pub = None

# Global commands
g_joint_cmd = JointCommand()
g_tip_cmd = PoseCommand()
g_mass_cmd = MassChange()

# Lock to make this code effectively single-threaded to avoid bugs
lock = Lock()

# Convert a ROS vector to numpy
def toNpVec(vec):
    return np.array([vec.x, vec.y, vec.z])

# Convert a numpy vec to ROS
def toRosVec(vec):
    rvec = Vector3()
    rvec.x = vec[0]
    rvec.y = vec[1]
    rvec.z = vec[2]
    return rvec

# Publish a message for a cetain publisher. This automatically updates last_msg_time
def publish_msg(pub, msg):
    global last_msg_time
    pub.publish(msg)
    last_msg_time = ros.Time.now()

# Callback to update robot current state
def robot_state_callback(data):
    global curr_state

    with lock:
        curr_state = data

def hist_mean(hist):
    np_hist = np.array([toNpVec(vec) for vec in hist])
    return toRosVec(np.mean(np_hist, axis=0))

def hist_stddev(hist):
    np_hist = np.array([toNpVec(vec) for vec in hist])
    return np.max(np.std(np_hist, axis=0))

# Callback to get new dets. Only saves when we want the det
def detection_callback(data, det_type):
    global detections, det_history
    det_history[det_type] = det_history[det_type][-20:] + [data.position]
    if (not want_detections[det_type]):
        det_history[det_type] = []
    with lock:
        if want_detections[det_type] and len(det_history[det_type]) >= 15 and hist_stddev(det_history[det_type]) < 0.005:
            detections[det_type] = hist_mean(det_history[det_type])

# Go to home, empty mass value
def startup():
    global joint_pub, tip_pub, mass_pub, active_state, multi_step
    
    

    if multi_step == 0:
        mass_msg = MassChange()
        publish_msg(mass_pub, mass_msg)

        tip_cmd = PoseCommand()
        tip_cmd.pose_follow = True
        tip_cmd.pos = deepcopy(curr_state.pos)
        tip_cmd.pos.z += 0.3
        publish_msg(tip_pub, tip_cmd)

        multi_step += 1

    elif multi_step == 1:
        joint_msg = JointCommand()
        publish_msg(joint_pub, joint_msg)
        active_state = wait_bottle_dets
        multi_step = 0

# Wait for a grabbable detection
def wait_bottle_dets():
    global detections, want_detections, active_state
    print('Waiting for det')
    want_detections[CAP_DET] = True
    want_detections[BAND_DET] = True
    want_detections[RIM_DET] = True
    # Found a cap (upright bottle), grab it
    if detections[CAP_DET] is not None:
        active_state = grab_bottle_cap
        want_detections[CAP_DET] = False
        want_detections[BAND_DET] = False
        want_detections[RIM_DET] = False
        detections[BAND_DET] = None
        detections[RIM_DET] = None
    # Fount a band and rim, upright it
    elif detections[BAND_DET] is not None and detections[RIM_DET] is not None and np.linalg.norm(toNpVec(detections[BAND_DET]) - toNpVec(detections[RIM_DET])) < 0.15:
        active_state = upright_bottle
        want_detections[CAP_DET] = False
        want_detections[BAND_DET] = False
        want_detections[RIM_DET] = False
        detections[CAP_DET] = None

# Pick up a bottle by the cap
def grab_bottle_cap():
    global detections, tip_pub, mass_pub, multi_step, active_state, g_tip_cmd, g_mass_cmd

    detections[CAP_DET].z = 0.18

    if math.sqrt(detections[CAP_DET].x**2 + detections[CAP_DET].y**2) < 0.75:
        detections[CAP_DET].z -= 0.02

    print('Grabbing by cap: Stage', multi_step)

    if multi_step == 0:
        g_tip_cmd = PoseCommand()
        g_tip_cmd.pose_follow = True
        g_tip_cmd.pos = deepcopy(detections[CAP_DET])
        g_tip_cmd.pos.z += 0.05

        g_mass_cmd = MassChange()
        g_mass_cmd.mass_status = g_mass_cmd.EMPTY
        multi_step += 1

    elif multi_step == 1:
        g_tip_cmd.pos = deepcopy(detections[CAP_DET])
        multi_step += 1

    elif multi_step == 2:
        g_tip_cmd.gripper = 1
        g_tip_cmd.wrist_roll = 3.14
        multi_step += 1

    elif multi_step == 3:
        g_mass_cmd.mass_status = g_mass_cmd.BOTTLE_TOP
        multi_step += 1

    elif multi_step == 4:
        g_tip_cmd.pos.z += 0.10
        g_tip_cmd.wrist_roll = 0
        multi_step = 0
        detections[CAP_DET] = None
        active_state = return_home

    publish_msg(tip_pub, g_tip_cmd)
    publish_msg(mass_pub, g_mass_cmd)


def upright_bottle():
    global detections, tip_pub, mass_pub, multi_step, active_state, g_tip_cmd, g_mass_cmd

    band = toNpVec(detections[BAND_DET])
    rim = toNpVec(detections[RIM_DET])
    band[2] = 0
    rim[2] = 0
    ori = rim - band
    ori /= np.linalg.norm(ori)
    angle = math.atan2(ori[1], ori[0])
    grab_point = band - 0.06 * ori

    grab_vec = toRosVec(grab_point)

    print('Uprighting bottle at angle', angle, ': Stage', multi_step)

    if multi_step == 0:
        g_tip_cmd = PoseCommand()
        g_tip_cmd.pose_follow = True
        g_tip_cmd.pos = deepcopy(grab_vec)
        g_tip_cmd.pos.z += 0.1
        g_tip_cmd.wrist_roll = angle

        g_mass_cmd = MassChange()
        g_mass_cmd.mass_status = g_mass_cmd.EMPTY
        multi_step += 1

    elif multi_step == 1:
        g_tip_cmd.pos.z -= 0.1
        multi_step += 1

    elif multi_step == 2:
        g_tip_cmd.gripper = 1
        multi_step += 1

    elif multi_step == 3:
        g_mass_cmd.mass_status = g_mass_cmd.BOTTLE_SIDE
        multi_step += 1

    elif multi_step == 4:
        g_tip_cmd.pos.z += 0.2
        g_tip_cmd.pos.y = 0
        g_tip_cmd.pos.x = 0.8
        multi_step += 1

    elif multi_step == 5:
        g_tip_cmd.wrist_roll = 0
        g_tip_cmd.wrist_angle = 1.57
        multi_step += 1

    elif multi_step == 6:
        g_tip_cmd.pos.z -= 0.15
        multi_step += 1

    elif multi_step == 7:
        g_mass_cmd.mass_status = g_mass_cmd.EMPTY
        multi_step += 1

    elif multi_step == 8:
        g_tip_cmd.gripper = 0
        multi_step += 1

    elif multi_step == 9:
        g_tip_cmd.pos.z += 0.25
        detections[BAND_DET] = None
        detections[RIM_DET] = None
        active_state = wait_bottle_dets
        multi_step = 0

    publish_msg(tip_pub, g_tip_cmd)
    publish_msg(mass_pub, g_mass_cmd)


def return_home():
    global joint_pub, active_state
    print('Returning home')
    joint_msg = JointCommand()
    joint_msg.gripper = 1
    publish_msg(joint_pub, joint_msg)
    active_state = throw_bottle

def throw_bottle():
    global throw_pub, active_state

    # Quick and dirty "did I grab the bottle?" check
    if curr_state.wrist_angle > 0.3:
        active_state = wait_bottle_dets
        return

    throw_cmd = ThrowCommand()
    throw_cmd.yaw_ang = random.uniform(-0.2, 0.4)
    throw_cmd.elbow_ang = 1.57
    throw_cmd.wrist_ang = 1.7
    throw_cmd.elbow_vel = 4.0
    throw_cmd.wrist_vel = 6.5
    throw_cmd.elbow_rest = 1.4
    throw_cmd.wrist_rest = 1.0
    publish_msg(throw_pub, throw_cmd)
    active_state = wait_bottle_dets

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

def do_nothing():
    pass

def shutdown():
    global active_state, tip_pub, mass_pub, g_tip_cmd

    print("Killed! Starting shutdown sequence...")
    active_state = do_nothing

    mass_cmd = MassChange()
    publish_msg(mass_pub, mass_cmd)

    g_tip_cmd = PoseCommand()
    g_tip_cmd.pose_follow = True
    g_tip_cmd.pos = deepcopy(curr_state.pos)
    g_tip_cmd.pos.z += 0.1

    publish_msg(tip_pub, g_tip_cmd)
    ros.sleep(ros.Duration(0.2))
    while(not curr_state.is_at_target):
        ros.sleep(ros.Duration(0.1))

    g_tip_cmd.pose_follow = False
    g_tip_cmd.pos.x = -0.11
    g_tip_cmd.pos.y = 0.45
    g_tip_cmd.pos.z = 0.1

    publish_msg(tip_pub, g_tip_cmd)
    ros.sleep(ros.Duration(0.2))
    while(not curr_state.is_at_target):
        ros.sleep(ros.Duration(0.1))

    g_tip_cmd.pose_follow = True
    g_tip_cmd.pos.z = -0.09

    publish_msg(tip_pub, g_tip_cmd)
    ros.sleep(ros.Duration(0.2))
    while(not curr_state.is_at_target):
        ros.sleep(ros.Duration(0.1))

    print("Shutdown complete!")

# Initialize and first time setup
def main():
    global joint_pub, tip_pub, throw_pub, mass_pub, curr_state, active_state, last_msg_time

    ros.init_node('logic_controller')
    ros.on_shutdown(shutdown)

    # Init subscribers
    ros.Subscriber("/robot_state", RobotState, robot_state_callback)
    ros.Subscriber("/bottle_cap_dets", Detection, detection_callback, CAP_DET)
    ros.Subscriber("/bottle_rim_det", Detection, detection_callback, RIM_DET)
    ros.Subscriber("/bottle_band_det", Detection, detection_callback, BAND_DET)

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