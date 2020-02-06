#!/usr/bin/env python
#
#   pymovetoexplicit.py
#
#   Continually (at 100Hz!) send commands to the robot, providing
#   explicit moves to goal locations - using a cubic spline.
#
from __future__ import print_function

import sys
import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64
from hw5code.msg import ThrowCommand

from threading import Lock

class SplineParams:
    def __init__(self):
        self.t0 = 0.0        # Cubic spline start time
        self.t1 = 0.0
        self.t2 = 0.0        # Cubic spline final time
        self.a  = 0.0        # Cubic spline t^0 parameter
        self.b  = 0.0        # Cubic spline t^1 parameter
        self.c  = 0.0        # Cubic spline t^2 parameter
        self.d  = 0.0        # Cubic spline t^3 parameter
        self.e  = 0.0        # Cubic spline t^3 parameter
        self.f  = 0.0        # Cubic spline t^3 parameter
    
    def setspline(self, homepos, throwpos, throwvel, tstart):
        # Pick a move time: use the time it would take to move the desired
        # distance at 50% of max speed (~1.5 rad/sec).  Also enforce a
        # 1sec min time.  Note this is arbitrary/approximate - we could
        # also compute the fastest possible time or pass as an argument.
        AVGSPEED = 1.5
        MINTIME  = 1.0
        t0 = tstart
        t1 = 2#math.fabs(throwpos - homepos) / AVGSPEED
        t2 = (t1) * 2

        hp = homepos
        yp = throwpos
        yv = throwvel

        # Lmao quintic splines are hard. This was not fun to transcribe from mathematica
        a = hp
        b = 0
        c = (hp*t1**4*(3*t1-5*t2) + a*(-3*t1**5+5*t1**4*t2-5*t1*t2**4+3*t2**5) + t2**4*(5*t1*yp-3*t2*yp-t1**2*yv+t1*t2*yv))/(t1**2*(t1-t2)**3*t2**2)
        d = (-2*c*t1**2*(t1**3-6*t1*t2**2+5*t2**3) + t2**2*(20*(t1-t2)*(a-yp) + t1*(4*t1-5*t2)*yv))/(t1**3*(3*t1-5*t2)*(t1-t2)*t2)
        e = (c*(-2*t1**5+5*t1**2*t2**3) + d*(-3*t1**5*t2+5*t1**3*t2**3) + 5*t2**3*(a-yp))/(t1**4*(4*t1-5*t2)*t2**2)
        f = (-a-t1**2*(c+t1*(d+e*t1)) + yp) / (t1**5)

        self.t0 = t0
        self.t1 = t1 + t0
        self.t2 = t2 + t0
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e
        self.f = f
    
    def get_point(self, t):
        """ Returns the value and derivative at the current time """
        if (t <= self.t0):
            r = 0.0
        elif (t >= self.t2):
            r = self.t2 - self.t0
        else:
            r = t  - self.t0

        cmdpos = self.a + self.b*r + self.c*r**2 + self.d*r**3 + self.e*r**4 + self.f*r**5
        cmdvel = self.b + 2.0*self.c*r + 3.0*self.d*r**2 + 4.0*self.e*r**3 + 5.0*self.f*r**4
        cmdtor = 0.0

        # if round(t * 1000) % 100 < 10:
        #     print("a: {}, b: {}, c: {}, d: {}, r: {}, cmdpos: {}, cmdvel: {}".format(
        #         self.a, self.b, self.c, self.d, r, cmdpos, cmdvel))

        return (cmdpos, cmdvel, cmdtor)
        

# Parameters for yaw and pitch splines
elbow_params = SplineParams()
wrist_params = SplineParams()

# Current states
current_arm_state = JointState()

# Current time (sec)              
t = 0.0

# Use a mutex so that parameters are accessed/read/set atomically. 
access_parameters_mutex = Lock()

# Current State Callback
def current_state_callback(msg):
    # This is important!!!
    global current_arm_state
    with access_parameters_mutex:
        current_arm_state = msg

def command_callback(msg):
    """ Update motion plans (splines) based on new commands """
    global elbow_params
    global wrist_params
    global current_arm_state
    global t
    with access_parameters_mutex:
        print("\n\nReceived command", msg, "\n Current state", current_arm_state, "\n")
        elbow_params.setspline(current_arm_state.position[0], msg.elbow_pos, msg.elbow_vel, t)
        wrist_params.setspline(current_arm_state.position[1], msg.wrist_pos, msg.wrist_vel, t)

#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pymovetoexplicit')

    # Publisher for joint commands
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    # Subscriber for current joint state
    cur_state_sub = rospy.Subscriber('/hebiros/robot/feedback/joint_state', JointState, current_state_callback)
    rospy.sleep(0.4)

    command_msg = JointState()
    command_msg.name.append('Boogaloo/elbow')    # Replace Family/Name
    command_msg.position.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(0.0)
    command_msg.name.append('Boogaloo/wrist')    # Replace Family/Name
    command_msg.position.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(0.0)
    command_msg.name.append('Boogaloo/shoulder')    # Replace Family/Name
    command_msg.position.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(0.0)
    command_msg.name.append('Boogaloo/gripper')    # Replace Family/Name
    command_msg.position.append(-.75)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(-.33)

    # Find the starting position and use as an offset for the sinusoid.
    # This will block, but that's appropriate as we don't want to start
    # until we have this information.  Make sure the joints are in the
    # same order in pydefinerobot as here - else things won't line up!
    current_arm_state = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState)
    

    # # Initialize the parameters and state variables.  Do this before
    # # the subscriber is activated (as it may run anytime thereafter).
    # # Set up the spline to move to zero (starting at t=1).
    initial_msg = ThrowCommand()
    initial_msg.elbow_pos = 0.0
    initial_msg.elbow_vel = 0.0
    initial_msg.wrist_pos = 0.0
    initial_msg.wrist_vel = 0.0

    # Reset position
    current_state_callback(initial_msg)

    t = 0.0

    # Subscriber user position goals
    # Now that the variables are valid, create/enable the subscriber
    # that (at any time hereafter) may read/update the settings.
    goal_sub = rospy.Subscriber('/goal', ThrowCommand, command_callback)


    # Create and run a servo loop at 100Hz until shutdown.
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt %f" % dt)

    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Compute the commands.  Make sure the spline parameters are
        # not changed in the middle of the calculations!
        with access_parameters_mutex:
            (elbowpos, elbowvel, elbowtor) = elbow_params.get_point(t)
            (wristpos, wristvel, wristtor) = wrist_params.get_point(t)

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = elbowpos # clamp the commanded 
        command_msg.velocity[0]  = elbowvel
        command_msg.effort[0]    = elbowtor

        command_msg.position[1]  = wristpos
        command_msg.velocity[1]  = wristvel
        command_msg.effort[1]    = wristtor
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
