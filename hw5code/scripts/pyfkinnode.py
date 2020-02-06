#!/usr/bin/env python
#
#   pyfkinnode.py
#
#   Compute the forward kinematics.
#
#   Subscribe: /joint_states      sensor_msgs/JointState
#   Publish:   /tippoint          geometry_msgs/PointStamped
#   Publish:   /tippose           geometry_msgs/PoseStamped
#
import rospy

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg   import JointState


#
#  Matrix Manipulation Utilities
#
def vec(x, y, z):
    return np.array([x, y, z])
def Rx(q):
    return Rotation.from_rotvec(q * vec(1, 0, 0))
def Ry(q):
    return Rotation.from_rotvec(q * vec(0, 1, 0))
def Rz(q):
    return Rotation.from_rotvec(q * vec(0, 0, 1))


#
#  FKin Node Class
#
class FKin:
    def __init__(self):
        # Create publishers to send the tip position/pose.
        self.pointPublisher = rospy.Publisher("/tippoint", PointStamped)
        self.posePublisher  = rospy.Publisher("/tippose",  PoseStamped)

        # Create a subscriber to listen to joint_states.
        rospy.Subscriber("joint_states", JointState, self.process)

    def process(self, msg):
        # Start before Joint 0.
        x = vec(0, 0, 0)
        R = Rz(0)

        # Rotate Joint 0 about Z axis.
        R = R * Rz(msg.position[0])

        # Shift to Joint 1.
        x = x + R.apply(vec(0.0, 0.05, 0.081))
        R = R * Rx(np.pi/2)

        # Rotate Joint 1 about Z axis.
        R = R * Rz(msg.position[1])

        # Shift to Joint 2.
        x = x + R.apply(vec(0.5, 0.0, 0.036))
        R = R;

        # Rotate Joint 2 about Z axis.
        R = R * Rz(msg.position[2])

        # Shift to the tip.
        x = x + R.apply(vec(0.5, 0.0, 0.0335))
        R = R * Ry(np.pi/2);

        # Publish the tip point.  Note that we declare the point as
        # given with respect to the world reference frame.
        point_msg = PointStamped()
        point_msg.header = msg.header;
        point_msg.header.frame_id = "world"

        point_msg.point.x = x[0]
        point_msg.point.y = x[1]
        point_msg.point.z = x[2]

        self.pointPublisher.publish(point_msg)

        # Publish the tip pose.  Note that we declare the pose as
        # given with respect to the world reference frame.
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = "world"

        pose_msg.pose.position.x = x[0]
        pose_msg.pose.position.y = x[1]
        pose_msg.pose.position.z = x[2]

        quat = R.as_quat()
        pose_msg.pose.orientation.x = quat[0];
        pose_msg.pose.orientation.y = quat[1];
        pose_msg.pose.orientation.z = quat[2];
        pose_msg.pose.orientation.w = quat[3];

        self.posePublisher.publish(pose_msg)


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('fkinnode')

    # Instantiate the FKin object.
    fkin = FKin()

    # Spin until shutdown.
    rospy.loginfo("Fkin: Running...")
    rospy.spin()
