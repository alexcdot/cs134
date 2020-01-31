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
from opencv_apps.msg import Rect, Face, FaceArrayStamped

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

PIXELS_PER_RADIAN = 700.0
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# We don't need a mutex here, as there is a single parameter (which
# hence will always be self-consistent!)
# access_parameters_mutex = Lock()


class ImplicitParams:
    def __init__(self):
        # Initialize the state variables.  Do this before
        # the subscriber is activated (as it may run anytime thereafter).
        self.cmdpos = 0.0
        self.cmdvel = 0.0
        self.cmdtor = 0.0
        self.goalpos = 0.0
    
    def set_goal(self, goalpos):
        self.goalpos = goalpos
    
    def set_pos(self, cmdpos):
        self.cmdpos = cmdpos
    
    def get_point(self, t, dt):
        """ Returns the desired velocity and position w.r.t. to the goal"""
        # Adjust the commands, effectively filtering the goal position
        # into the command position.  Note we only use a single
        # parameter (goalpos) which we read just once, so there is no
        # chance of self-inconsistency.  I.e. we don't need to mutex!
        TIMECONSTANT = 0.3		# Convergence time constant
        LAMBDA       = 2.0/TIMECONSTANT # Convergence rate
        MAXVELOCITY  = 1.5              # Velocity magnitude limit

        cmdacc = - 1.4 * LAMBDA * self.cmdvel - LAMBDA*LAMBDA* (
            self.cmdpos - self.goalpos)
        self.cmdvel = self.cmdvel + dt * cmdacc
        if   (self.cmdvel >  MAXVELOCITY):
            self.cmdvel =  MAXVELOCITY
        elif (self.cmdvel < -MAXVELOCITY):
            self.cmdvel = -MAXVELOCITY
        self.cmdpos = self.cmdpos + dt * self.cmdvel

        self.cmdtor = 0.0
        return (self.cmdpos, self.cmdvel, self.cmdtor)


# Parameters for yaw and pitch splines
yaw_params = ImplicitParams()
pitch_params = ImplicitParams()

# Current states
current_arm_state = JointState()

# Current State Callback
def current_state_callback(msg):
    global current_arm_state
    current_arm_state = msg

#
#   Goal Subscriber Callback
#
#   This message is of type std_msgs::Float64, i.e. it contains only
#   one number.  Use that as a new goal position.
#
def yaw_pitch_callback(msg):
    # Simply save the new goal position.
    global yaw_params, pitch_params

    min_yaw = -2
    max_yaw = 2
    yaw_params.set_goal(min(max(msg.yaw, min_yaw), max_yaw))
    
    pitch_params.set_goal(msg.pitch)

    # Report.
    rospy.loginfo("Moving goal to yaw: %6.3frad, pitch: %6.3frad"
                  % (msg.yaw, msg.pitch))


def faces_callback(msg):
    # Transform the face coordinates to rotations
    """
    /FaceArrayStamped "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
    faces:
    - face:
        x: 0.0
        y: 0.0
        width: 0.0
        height: 0.0
    eyes:
    - x: 0.0
        y: 0.0
        width: 0.0
        height: 0.0
    label: ''
    confidence: 0.0" 
    """
    # call yaw_pitch_callback with a YawPitchCommand
    global current_arm_state
    if len(msg.faces) > 0:
        # find the face with the highest confidence
        face_i = 0
        face_c = msg.faces[0].face.height
        for i in range(1, len(msg.faces)):
            c = msg.faces[i].face.height
            if c > face_c:
                face_i = i
                face_c = c

        

        # create a new goal
        face = msg.faces[face_i].face

        msg = YawPitchCommand()

        yaw = current_arm_state.position[0]
        pitch = current_arm_state.position[1]
        
        msg.yaw = yaw - (face.x - IMAGE_WIDTH / 2) / PIXELS_PER_RADIAN
        msg.pitch = pitch + (face.y - IMAGE_HEIGHT / 2) / PIXELS_PER_RADIAN

        yaw_pitch_callback(msg)

#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pymovetoimplicit')

    # Create a publisher to send commands to the robot.  Add some time
    # for the subscriber to connect so we don't loose initial
    # messages.  Also initialize space for the message data.

    # Publisher for joint commands
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    # Subscriber for current joint state
    cur_state_sub = rospy.Subscriber('/hebiros/robot/feedback/joint_state', JointState, current_state_callback)
    face_sub = rospy.Subscriber('/detector/faces', FaceArrayStamped, faces_callback)
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

    t      = 0.0
    yaw_params.set_pos(current_arm_state.position[0])
    pitch_params.set_pos(current_arm_state.position[1])
    # Initialize the parameters and state variables.  Do this before
    # the subscriber is activated (as it may run anytime thereafter).
    # Set up the spline to move to zero (starting at t=1).
    initial_msg = YawPitchCommand()
    initial_msg.yaw = 0
    initial_msg.pitch = 0
    # Reset position
    yaw_pitch_callback(initial_msg)

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

        (yawpos, yawvel, yawtor) = yaw_params.get_point(t, dt)
        (pitchpos, pitchvel, pitchtor) = pitch_params.get_point(t, dt)

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = yawpos # clamp the commanded 
        command_msg.velocity[0]  = yawvel
        command_msg.effort[0]    = yawtor

        command_msg.position[1]  = pitchpos
        command_msg.velocity[1]  = pitchvel
        command_msg.effort[1]    = pitchtor
        if round(t * 1000) % 100 < 10:
            print("time:", t, " command:", command_msg)
            print("yaw goals:", yaw_params.goalpos, "pitch goals:", pitch_params.goalpos)
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
