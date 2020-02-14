#include "armcontroller.hpp"

int main(int argc, char **argv) {
    // Initialize the basic ROS node.
    ros::init(argc, argv, "fkinnode");
    
    ArmController controller = ArmController();

    return 0;
}

ArmController::ArmController() {
    kinematics_ = Kinematics();

    arm_state_sub_ = nh_.subscribe("joint_states", 10, &ArmController::processJointState, this);
    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/tippoint", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tippose", 10);

    ROS_INFO("Fkin: Running...");
    ros::spin();
}

void ArmController::processJointState(const sensor_msgs::JointState::ConstPtr& msg) {
    Vector6d joints = Vector6d::Zero();
    for (int i = 0; i < 6; i++) {
        joints(i) = msg->position[i];
    }
    joints(5) = 0;
    FKinResult res = kinematics_.runForwardKinematics(joints);

    // Create point message
    geometry_msgs::PointStamped point;
    point.header.frame_id = "world";
    point.point.x = res.pose(0);
    point.point.y = res.pose(1);
    point.point.z = res.pose(2);
    point_pub_.publish(point);

    // Create pose message
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = res.pose(0);
    pose.pose.position.y = res.pose(1);
    pose.pose.position.z = res.pose(2);
    pose.pose.orientation.x = res.orientation.x();
    pose.pose.orientation.y = res.orientation.y();
    pose.pose.orientation.z = res.orientation.z();
    pose.pose.orientation.w = res.orientation.w();
    pose_pub_.publish(pose);
    
    Vector6d guess;
    guess << -1.57, 0, 0, 0, 0, 0;
    Vector6d rebuilt = kinematics_.runInverseKinematics(res.pose, guess);
}

