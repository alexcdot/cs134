/*
**   fkinnode.cpp
**
**   Compute the forward kinematics.
**
**   Subscribe: /joint_states      sensor_msgs/JointState
**   Publish:   /tippoint          geometry_msgs/PointStamped
**   Publish:   /tippose           geometry_msgs/PoseStamped
*/
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


/*
**   Global Variables.  Make the publishers available to the callback.
*/
ros::Publisher pointPublisher;
ros::Publisher posePublisher;


/*
**   Abbreviations for Eigen matrix manipulation.
*/
#define vec(x,y,z)  (Eigen::Vector3d((x), (y), (z)))
#define Rx(q)       (Eigen::AngleAxisd((q), vec(1.0, 0.0, 0.0)))
#define Ry(q)       (Eigen::AngleAxisd((q), vec(0.0, 1.0, 0.0)))
#define Rz(q)       (Eigen::AngleAxisd((q), vec(0.0, 0.0, 1.0)))


/*
**   Jointstate Subscriber Callback
*/
void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msgptr)
{ 
  Eigen::Vector3d  x;		// Position w.r.t. world
  Eigen::Matrix3d  R;		// Orientation w.r.t. world

  // Start before Joint 0.
  x.setZero();
  R.setIdentity();

  // Rotate Joint 0 about Z axis.
  R = R * Rz(msgptr->position[0]);

  // Shift to Joint 1.
  x = x + R * vec(0.0, 0.05, 0.081);
  R = R * Rx(M_PI/2.0);

  // Rotate Joint 1 about Z axis.
  R = R * Rz(msgptr->position[1]);

  // Shift to Joint 2.
  x = x + R * vec(0.5, 0.0, 0.036);
  R = R;

  // Rotate Joint 2 about Z axis.
  R = R * Rz(msgptr->position[2]);

  // Shift to the tip.
  x = x + R * vec(0.5, 0.0, 0.0335);
  R = R * Ry(M_PI/2.0);


  // Publish the tip point.  Note that we declare the point as given
  // with respect to the world reference frame.
  geometry_msgs::PointStamped  point_msg;
  point_msg.header = msgptr->header;
  point_msg.header.frame_id = "world";

  point_msg.point.x = x(0);
  point_msg.point.y = x(1);
  point_msg.point.z = x(2);

  pointPublisher.publish(point_msg);

  // Publish the tip pose.  Note that we declare the pose as given
  // with respect to the world reference frame.
  geometry_msgs::PoseStamped  pose_msg;
  pose_msg.header = msgptr->header;
  pose_msg.header.frame_id = "world";

  pose_msg.pose.position.x = x(0);
  pose_msg.pose.position.y = x(1);
  pose_msg.pose.position.z = x(2);

  Eigen::Quaterniond  quat = Eigen::Quaterniond(R);
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();
  pose_msg.pose.orientation.w = quat.w();

  posePublisher.publish(pose_msg);
}


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "fkinnode");
  ros::NodeHandle n;

  // Create publishers to send the tip position/pose.
  pointPublisher = n.advertise<geometry_msgs::PointStamped>("/tippoint", 10);
  posePublisher  = n.advertise<geometry_msgs::PoseStamped>("/tippose", 10);

  // Create a subscriber to listen to joint_states.
  ros::Subscriber jointstateSubscriber
    = n.subscribe("joint_states", 10, jointstateCallback);

  // Spin until shutdown.
  ROS_INFO("Fkin: Running...");
  ros::spin();

  return 0;
}
