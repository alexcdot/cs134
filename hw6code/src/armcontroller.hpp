#ifndef __ARM_CONTROLLER__
#define __ARM_CONTROLLER__

#include "common.hpp"
#include "splinemanager.hpp"
#include "kinematics.hpp"

#include "boogaloo/JointCommand.h"
#include "boogaloo/PoseCommand.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

enum class ArmControllerState {
    STOPPED,
    MOVING_JOINT,
    MOVING_POINT
};

class ArmController {
    private: 
    ArmControllerState current_state_;
    Vector6d current_joint_pos_;
    Vector6d current_joint_vel_;
    Vector6d current_pose_pos_;
    Vector6d current_pose_vel_;
    SplineManager spline_managers_[6];
    Kinematics kinematics_;

    ros::NodeHandle nh_;

    ros::Subscriber arm_state_sub_;
    ros::Subscriber tip_goal_sub_;
    ros::Subscriber joint_goal_sub_;
    
    ros::Publisher point_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher joint_command_pub_;

    public:

    ArmController();

    void processJointCommand(const boogaloo::JointCommand::ConstPtr& msg);
    void processPoseCommand(const boogaloo::PoseCommand::ConstPtr& msg);

    void processJointState(const sensor_msgs::JointState::ConstPtr& msg);

    void runController(const ros::TimerEvent& time);
};

#endif