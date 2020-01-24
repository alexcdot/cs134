#!/usr/bin/env python
#
#   pymovetoimplicit.py
#
#   Continually (at 100Hz!) send commands to the robot, providing
#   implicit moves to goal locations - using a filter.
#
import sys
import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64
from assignment3.msg import YawPitchCommand

from threading import Lock


#
#   Global Variables.  We use global variables so the callbacks can
#   see the state and pass information to the main (timer) loop.
#
goalpos = 0.0		# Goal position
                                                        
t      = 0.0            # Current time (sec)
cmdpos = 0.0            # Current cmd position (rad)
cmdvel = 0.0            # Current cmd velocity (rad/sec)
cmdtor = 0.0            # Current cmd torque (Nm)

# We don't need a mutex here, as there is a single parameter (which
# hence will always be self-consistent!)
# access_parameters_mutex = Lock()

# Current states
current_arm_state = JointState()

# Current State Callback
def current_state_callback(msg):
    with access_parameters_mutex:
        current_arm_state = msg

#
#   Goal Subscriber Callback
#
#   This message is of type std_msgs::Float64, i.e. it contains only
#   one number.  Use that as a new goal position.
#
def goalCallback(msg):
    # Simply save the new goal position.
    global goalpos
    goalpos = msg.data

    # Report.
    rospy.loginfo("Moving goal to %6.3frad" % goalpos)


class ImplicitParams:
    def __init__(self):
        self.cmdpos = 0.0
        self.cmdvel = 0.0
        self.cmd
    
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
        with access_parameters_mutex:
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

        return (cmdpos, cmdvel, 0.0)



def updateDesiredYawPitch(msg):
    global current_arm_state
    global t


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pymovetoimplicit')

    # Create a publisher to send commands to the robot.  Add some time
    # for the subscriber to connect so we don't loose initial
    # messages.  Also initialize space for the message data.
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    rospy.sleep(0.4)

    command_msg = JointState()
    command_msg.name.append('Dwarfs/Doc')    # Replace Family/Name
    command_msg.position.append(0.0)
    command_msg.velocity.append(0.0)
    command_msg.effort.append(0.0)

    # Find the starting position and use as an offset for the sinusoid.
    # This will block, but that's appropriate as we don't want to start
    # until we have this information.  Make sure the joints are in the
    # same order in pydefinerobot as here - else things won't line up!
    msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState);

    # Initialize the parameters and state variables.  Do this before
    # the subscriber is activated (as it may run anytime thereafter).
    goalpos = 0.0

    t      = 0.0
    cmdpos = msg.position[0]
    cmdvel = 0.0
    cmdtor = 0.0

    # Now that the variables are valid, create/enable the subscriber
    # that (at any time hereafter) may read/update the settings.
    rospy.Subscriber("goal", Float64, goalCallback)

    # Create and run a servo loop at 100Hz until shutdown.
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt %f" % dt)

    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        with access_parameters_mutex:
            # Adjust the commands, effectively filtering the goal position
            # into the command position.  Note we only use a single
            # parameter (goalpos) which we read just once, so there is no
            # chance of self-inconsistency.  I.e. we don't need to mutex!
            TIMECONSTANT = 0.7		# Convergence time constant
            LAMBDA       = 1.0/TIMECONSTANT # Convergence rate
            MAXVELOCITY  = 1.5              # Velocity magnitude limit

            cmdacc = - 1.4 * LAMBDA * cmdvel - LAMBDA*LAMBDA* (cmdpos - goalpos)
            cmdvel = cmdvel + dt * cmdacc
            if   (cmdvel >  MAXVELOCITY):
                cmdvel =  MAXVELOCITY
            elif (cmdvel < -MAXVELOCITY):
                cmdvel = -MAXVELOCITY
            cmdpos = cmdpos + dt * cmdvel

            cmdtor = 0.0

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = cmdpos
        command_msg.velocity[0]  = cmdvel
        command_msg.effort[0]    = cmdtor
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
