#!/usr/bin/env python
#
#   pymovetoexplicit.py
#
#   Continually (at 100Hz!) send commands to the robot, providing
#   explicit moves to goal locations - using a cubic spline.
#
import sys
import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64
from assignment3.msg import YawPitchCommand

from threading import Lock

class SplineParams:
    def __init__(self):
        self.t0 = 0.0        # Cubic spline start time
        self.tf = 0.0        # Cubic spline final time
        self.a  = 0.0        # Cubic spline t^0 parameter
        self.b  = 0.0        # Cubic spline t^1 parameter
        self.c  = 0.0        # Cubic spline t^2 parameter
        self.d  = 0.0        # Cubic spline t^3 parameter
    
    def setspline(self, goalpos, startpos, startvel, tstart):
        # Pick a move time: use the time it would take to move the desired
        # distance at 50% of max speed (~1.5 rad/sec).  Also enforce a
        # 1sec min time.  Note this is arbitrary/approximate - we could
        # also compute the fastest possible time or pass as an argument.
        AVGSPEED = 1.5
        MINTIME  = 1.0
        tmove = math.fabs(goalpos - startpos) / AVGSPEED
        if (tmove < MINTIME):
            tmove = MINTIME

        # Set the cubic spline parameters.  Make sure the main code (timer)
        # doesn't access until we're done.
        self.t0 = tstart
        self.tf = tstart + tmove
        self.a  = startpos
        self.b  = startvel
        self.c  = ( 3.0 * (goalpos - startpos) / tmove + 2.0 * startvel) / tmove
        self.d  = (-2.0 * (goalpos - startpos) / tmove - 3.0 * startvel) / (tmove*tmove)
    
    def get_point(self, t):
        """ Returns the value and derivative at the current time """
        if (t <= self.t0):
            r = 0.0
        elif (t >= self.tf):
            r = self.tf - self.t0
        else:
            r = t  - self.t0

        cmdpos = self.a + self.b*r + self.c*r*r + self.d*r*r*r
        cmdvel = self.b + 2.0*self.c*r + 3.0*self.d*r*r
        cmdtor = 0.0

        if round(t * 1000) % 100 < 10:
            print("a: {}, b: {}, c: {}, d: {}, r: {}, cmdpos: {}, cmdvel: {}".format(
                self.a, self.b, self.c, self.d, r, cmdpos, cmdvel))

        return (cmdpos, cmdvel, cmdtor)
        

# Parameters for yaw and pitch splines
yaw_params = SplineParams()
pitch_params = SplineParams()

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

def yaw_pitch_callback(msg):
    """ Update motion plans (splines) based on new commands """
    global yaw_params
    global pitch_params
    global current_arm_state
    global t
    min_yaw = -2
    max_yaw = 2
    with access_parameters_mutex:
        print("\n\nReceived command", msg, "\n Current state", current_arm_state, "\n")
        clamped_yaw  = min(max(msg.yaw, min_yaw), max_yaw) # clamp the commanded 
        yaw_params.setspline(clamped_yaw, current_arm_state.position[0],
                              current_arm_state.velocity[0], t)
        pitch_params.setspline(msg.pitch, current_arm_state.position[1],
                               current_arm_state.velocity[1], t)


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
    command_msg.name.append('Boogaloo/yaw')    # Replace Family/Name
    command_msg.position.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(0.0)
    command_msg.name.append('Boogaloo/pitch')    # Replace Family/Name
    command_msg.position.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(0.0)

    # Find the starting position and use as an offset for the sinusoid.
    # This will block, but that's appropriate as we don't want to start
    # until we have this information.  Make sure the joints are in the
    # same order in pydefinerobot as here - else things won't line up!
    current_arm_state = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState)
    

    # Initialize the parameters and state variables.  Do this before
    # the subscriber is activated (as it may run anytime thereafter).
    # Set up the spline to move to zero (starting at t=1).
    initial_msg = YawPitchCommand()
    initial_msg.yaw = 0.0
    initial_msg.pitch = 0.0

    # Reset position
    yaw_pitch_callback(initial_msg)

    t = 0.0

    # Subscriber user position goals
    # Now that the variables are valid, create/enable the subscriber
    # that (at any time hereafter) may read/update the settings.
    goal_sub = rospy.Subscriber('/goal', YawPitchCommand, yaw_pitch_callback)


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
            (yawpos, yawvel, yawtor) = yaw_params.get_point(t)
            (pitchpos, pitchvel, pitchtor) = pitch_params.get_point(t)
            if round(t * 1000) % 100 < 10:
                print("yaw command:", t, yawpos, pitchpos)

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = yawpos # clamp the commanded 
        command_msg.velocity[0]  = yawvel
        command_msg.effort[0]    = yawtor

        command_msg.position[1]  = pitchpos
        command_msg.velocity[1]  = pitchvel
        command_msg.effort[1]    = pitchtor
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
