#!/usr/bin/env python
#
#   pysendcommands.py
#
#   Continually (at 100Hz!) send commands to the robot
#   (in hebiros_node).

import sys
import rospy
import math
import numpy as np

from sensor_msgs.msg import JointState

def deg_to_rad(deg):
    return deg / 180.0 * math.pi

if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pysendcommands')

    state_msg = JointState()

    def sub_callback(msg):
        global state_msg
        state_msg = msg

    # Create a publisher to send commands to the robot.  Also
    # initialize space for the message data.
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=100)
    sub = rospy.Subscriber('/hebiros/robot/feedback/joint_state', JointState, queue_size=100, callback=sub_callback)

    command_msg = JointState()
    command_msg.name.append('Tiger/2')    # Replace Family/Name
    command_msg.position.append(0)
    # command_msg.velocity.append(0)
    # command_msg.effort.append(0)
    Hz = 1
    num_cycles = 2
    update_rate = 100

    position_data = np.empty((int((num_cycles + 1)/ Hz * update_rate), 3))

    # Create a servo loop at 100Hz.
    servo = rospy.Rate(update_rate)
    dt    = servo.sleep_dur.to_sec()

    # Run the servo loop until shutdown.
    rospy.loginfo("Running the servo loop with dt %f" % dt)
                  
    starttime = rospy.Time.now()
    i = 0
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Compute the commands.
        cmdpos = math.sin(t * (2 * math.pi) * Hz) * deg_to_rad(45)
        cmdvel = 0.0
        cmdtor = 0.0

        try:
            if len(state_msg.position) == 0:
                print("continued")
                continue
        except Exception:
            print("error", state_msg.position)
            raise Exception
        

        # interpolate from the start position
        interp = max(0,min(t, 10)) / 10
        cmdpos = interp * cmdpos + (1 - interp) * state_msg.position[0]

        if i < position_data.shape[0]:
            position_data[i,:] = [t, state_msg.position[0], cmdpos]
        else:
            break

        print("time:", t)
        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[0]  = cmdpos
        #command_msg.velocity[0]  = cmdvel
        # command_msg.effort[0]    = cmdtor
        pub.publish(command_msg)
        
        i += 1

        # Wait for the next turn.
        servo.sleep()

    np.savetxt('{}-Hz-recording.csv'.format(Hz), position_data, delimiter=',',
               header='time(s),actual_theta(rad),desired_theta(rad)', comments='')