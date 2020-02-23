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
    MOVING_JOINT,
    MOVING_POINT,
    FLOATING
};

struct PosVelPair {
    Vector6d pos;
    Vector6d vel;
};

class ArmController {
    private: 
    ArmControllerState current_state_;
    Vector6d current_joint_pos_;
    Vector6d current_joint_vel_;
    Vector6d current_pose_pos_;
    Vector6d current_pose_vel_;
    Vector6d feedback_joint_pos_;
    Vector6d feedback_joint_vel_;
    SplineManager spline_managers_[6];
    Kinematics kinematics_;

    ros::NodeHandle nh_;

    ros::Subscriber arm_state_sub_;
    ros::Subscriber tip_goal_sub_;
    ros::Subscriber joint_goal_sub_;
    ros::Subscriber feedback_sub_;
    
    ros::Publisher point_pub_;
    ros::Publisher pose_pub_;

    ros::Publisher joint_command_pub_;
    ros::Publisher joint_state_pub_;

    ros::Timer run_timer_;

    ros::Time start_time_;

    public:

    ArmController();

    void processJointCommand(const boogaloo::JointCommand::ConstPtr& msg);
    void processPoseCommand(const boogaloo::PoseCommand::ConstPtr& msg);
    void processFeedback(const sensor_msgs::JointState::ConstPtr& msg);

    void runController(const ros::TimerEvent& time);

    void setSplines(Vector6d goal_pos, Vector6d start_pos, Vector6d start_vel, ros::Time t_start);
    PosVelPair getSplinePoints(ros::Time time);
};

#endif