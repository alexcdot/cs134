#include "armcontroller.hpp"

int main(int argc, char **argv) {
    // Initialize the basic ROS node.
    ros::init(argc, argv, "fkinnode");
    
    ArmController controller = ArmController();

    return 0;
}

ArmController::ArmController() {
    kinematics_ = Kinematics();

    robot_state_pub_ = nh_.advertise<boogaloo::RobotState>(
        "/robot_state", 10
    );

    start_time_ = ros::Time::now();

    // Get initial position
    sensor_msgs::JointState::ConstPtr msgptr;
    msgptr = ros::topic::waitForMessage<sensor_msgs::JointState>(
        "/hebiros/robot/feedback/joint_state", nh_
    );
    if (msgptr == NULL) {
        ROS_ERROR("Failed to receive topic /hebiros/robot/feedback/joint_state");
        exit(1);
    }
    this->processFeedback(msgptr);

    FKinResult initial_pose = kinematics_.runForwardKinematics(feedback_joint_pos_);
    
    current_joint_pos_ = feedback_joint_pos_;
    current_joint_vel_ = feedback_joint_vel_;
    current_pose_pos_ = initial_pose.pose;
    current_pose_vel_ = initial_pose.jacobian * feedback_joint_vel_;

    current_state_ = ArmControllerState::FLOATING;
    this->setSplinesJoint(current_joint_pos_, current_joint_vel_, current_joint_pos_, ros::Time::now());

    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/tippoint", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tippose", 10);
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/hebiros/robot/command/joint_state", 10
    );
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/commanded_joints", 10
    );

    // Goal subscribers
    joint_goal_sub_ = nh_.subscribe("/joint_goal", 10, &ArmController::processJointCommand, this);
    tip_goal_sub_ = nh_.subscribe("/tip_goal", 10, &ArmController::processPoseCommand, this);
    throw_goal_sub_ = nh_.subscribe("/throw_goal", 10, &ArmController::processThrowCommand, this);
    feedback_sub_ = nh_.subscribe("/hebiros/robot/feedback/joint_state", 10, &ArmController::processFeedback, this);
    mass_sub_ = nh_.subscribe("/mass_updates", 10, &Kinematics::updateMassStatus, &kinematics_);

    run_timer_ = nh_.createTimer(ros::Duration(0.01), &ArmController::runController, this);
    run_timer_.start();

    ros::spin();
}

void ArmController::runController(const ros::TimerEvent& time) {
    ros::Time t = time.current_real;

    // Joint state to command
    sensor_msgs::JointState joint_state;
    // Fkin result to draw gripper point and pose
    FKinResult fkin_result;

    if (current_state_ == ArmControllerState::MOVING_POINT) {
        // Get tip pos/vel
        PosVelPair tip = this->getSplinePoints(t);

        // Get joints using ikin
        Vector6d target_joint = kinematics_.runInverseKinematics(tip.pos);

        // Get joint vel using fkin jacobian
        FKinResult fkin = kinematics_.runForwardKinematics(target_joint);
        Vector6d target_joint_vel = fkin.jacobian.inverse() * tip.vel;
        Vector6d target_joint_torques = kinematics_.getExpectedJointTorques(target_joint);

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(target_joint, target_joint_vel, target_joint_torques);
        fkin_result = fkin;

        // Update current values
        current_joint_pos_ = target_joint;
        current_joint_vel_ = target_joint_vel;
        current_pose_pos_ = tip.pos;
        current_pose_vel_ = tip.vel;
    }
    else if (current_state_ == ArmControllerState::MOVING_JOINT) {
        // Get joint pos/vel
        PosVelPair joints = this->getSplinePoints(t);
        Vector6d joint_torques = kinematics_.getExpectedJointTorques(joints.pos);

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(joints.pos, joints.vel, joint_torques);

        // Run fkin to get jacobian for later
        FKinResult fkin = kinematics_.runForwardKinematics(joints.pos);

        fkin_result = fkin;

        // Update current values
        current_joint_pos_ = joints.pos;
        current_joint_vel_ = joints.vel;
        current_pose_pos_ = fkin.pose;
        current_pose_vel_ = fkin.jacobian * joints.vel;
    }
    else if (current_state_ == ArmControllerState::FLOATING) {
        current_robot_state_.is_at_target = true;

        Vector6d joint_torques = kinematics_.getExpectedJointTorques(feedback_joint_pos_);
        double percent = min(1.0, (ros::Time::now() - start_time_).toSec() / 5);
        joint_torques = joint_torques * percent;

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(feedback_joint_pos_, Vector6d::Zero(), joint_torques);

        
        FKinResult fkin = kinematics_.runForwardKinematics(feedback_joint_pos_);
        fkin_result = fkin;

        // Update current values
        current_joint_pos_ = feedback_joint_pos_;
        current_joint_vel_ = feedback_joint_vel_;
        current_pose_pos_ = fkin.pose;
        current_pose_vel_ = fkin.jacobian * feedback_joint_vel_;
    }
    else if (current_state_ == ArmControllerState::WINDUP) {
        // Get joint pos/vel
        PosVelPair joints = this->getSplinePoints(t);
        Vector6d joint_torques = kinematics_.getExpectedJointTorques(joints.pos);

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(joints.pos, joints.vel, joint_torques);

        // Run fkin to get jacobian for later
        FKinResult fkin = kinematics_.runForwardKinematics(joints.pos);

        fkin_result = fkin;

        // Update current values
        current_joint_pos_ = joints.pos;
        current_joint_vel_ = joints.vel;
        current_pose_pos_ = fkin.pose;
        current_pose_vel_ = fkin.jacobian * joints.vel;

        if (current_robot_state_.is_at_target) {
            setSplinesThrow(current_joint_pos_, current_joint_vel_, requested_throw_pos_, requested_throw_vel_, ros::Time::now());
            current_state_ = ArmControllerState::THROW;
        }

        current_robot_state_.is_at_target = false;
    }
    else if (current_state_ == ArmControllerState::THROW) {
        // Get joint pos/vel
        PosVelPair joints = this->getSplinePoints(t);
        Vector6d joint_torques = kinematics_.getExpectedJointTorques(joints.pos);

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(joints.pos, joints.vel, joint_torques);

        // Run fkin to get jacobian for later
        FKinResult fkin = kinematics_.runForwardKinematics(joints.pos);

        fkin_result = fkin;

        // Update current values
        current_joint_pos_ = joints.pos;
        current_joint_vel_ = joints.vel;
        current_pose_pos_ = fkin.pose;
        current_pose_vel_ = fkin.jacobian * joints.vel;

        if (current_robot_state_.is_at_target) {
            setSplinesJoint(requested_pause_pos_, Vector6d::Zero(), requested_pause_pos_, ros::Time::now());
            current_state_ = ArmControllerState::PAUSE;
            boogaloo::MassChange new_mass = boogaloo::MassChange();
            new_mass.mass_status = boogaloo::MassChange::EMPTY;
            kinematics_.updateMassStatus(new_mass);
        }

        current_robot_state_.is_at_target = false;
    }
    else if (current_state_ == ArmControllerState::PAUSE) {
        // Get joint pos/vel
        PosVelPair joints = this->getSplinePoints(t);
        Vector6d joint_torques = kinematics_.getExpectedJointTorques(joints.pos);

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(joints.pos, joints.vel, joint_torques);

        // Run fkin to get jacobian for later
        FKinResult fkin = kinematics_.runForwardKinematics(joints.pos);

        fkin_result = fkin;

        // Update current values
        current_joint_pos_ = joints.pos;
        current_joint_vel_ = joints.vel;
        current_pose_pos_ = fkin.pose;
        current_pose_vel_ = fkin.jacobian * joints.vel;

        if (current_robot_state_.is_at_target) {
            Vector6d home = Vector6d::Zero();
            setSplinesJoint(current_joint_pos_, current_joint_vel_, home, ros::Time::now());
            current_state_ = ArmControllerState::MOVING_JOINT;
        }

        current_robot_state_.is_at_target = false;
    }

    joint_state_pub_.publish(joint_state);
    joint_command_pub_.publish(kinematics_.toHebi(joint_state));

    // Publish point and pose
    geometry_msgs::PointStamped point;
    point.header.frame_id = "world";
    point.point.x = fkin_result.pose(0);
    point.point.y = fkin_result.pose(1);
    point.point.z = fkin_result.pose(2);
    point_pub_.publish(point);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = fkin_result.pose(0);
    pose.pose.position.y = fkin_result.pose(1);
    pose.pose.position.z = fkin_result.pose(2);
    pose.pose.orientation.x = fkin_result.orientation.x();
    pose.pose.orientation.y = fkin_result.orientation.y();
    pose.pose.orientation.z = fkin_result.orientation.z();
    pose.pose.orientation.w = fkin_result.orientation.w();
    pose_pub_.publish(pose);
}

void ArmController::processJointCommand(const boogaloo::JointCommand::ConstPtr& msg) {
    ros::Time curr_time = ros::Time::now();
    Vector6d joint_command;
    joint_command << msg->yaw, msg->shoulder, msg->elbow,
                     msg->wrist, msg->twist, msg->gripper;

    if (msg->pose_follow) {
        current_state_ = ArmControllerState::MOVING_POINT;
        FKinResult fkin = kinematics_.runForwardKinematics(joint_command);
        this->setSplinesPose(current_pose_pos_, current_pose_vel_, fkin.pose, curr_time);
    } 
    else {
        current_state_ = ArmControllerState::MOVING_JOINT;
        this->setSplinesJoint(current_joint_pos_, current_joint_vel_, joint_command, curr_time);
    }
}

void ArmController::processPoseCommand(const boogaloo::PoseCommand::ConstPtr& msg) {
    ros::Time curr_time = ros::Time::now();
    Vector6d pose_command;
    pose_command << msg->pos.x, msg->pos.y, msg->pos.z,
                    msg->wrist_angle, msg->wrist_roll, msg->gripper;

    if (msg->pose_follow) {
        current_state_ = ArmControllerState::MOVING_POINT;
        this->setSplinesPose(current_pose_pos_, current_pose_vel_, pose_command, curr_time);
    } 
    else {
        current_state_ = ArmControllerState::MOVING_JOINT;
        Vector6d joint_command = kinematics_.runInverseKinematics(pose_command);
        this->setSplinesJoint(current_joint_pos_, current_joint_vel_, joint_command, curr_time);
    }
}

void ArmController::processThrowCommand(const boogaloo::ThrowCommand::ConstPtr& msg) {
    ros::Time curr_time = ros::Time::now();

    Vector6d windup_joint_pos;
    windup_joint_pos <<
        msg->yaw_ang,
        0.0,
        -3*PI/4,
        0.0,
        1.57,
        1.0;
    current_state_ = ArmControllerState::WINDUP;
    setSplinesJoint(current_joint_pos_, current_joint_vel_, windup_joint_pos, curr_time);

    requested_throw_pos_ <<
        msg->yaw_ang,
        0.0,
        msg->elbow_ang - PI/2,
        -(msg->wrist_ang - msg->elbow_ang) - PI/2,
        1.57,
        0.0;

    requested_throw_vel_ <<
        0.0,
        0.0,
        msg->elbow_vel,
        -msg->wrist_vel,
        0.0,
        0.0;

    requested_pause_pos_ <<
        msg->yaw_ang,
        0.0,
        msg->elbow_rest - PI/2,
        -(msg->wrist_rest - msg->elbow_rest) - PI/2,
        1.57,
        0.0;
}

void ArmController::processFeedback(const sensor_msgs::JointState::ConstPtr& msg) {
    sensor_msgs::JointState real_state = kinematics_.fromHebi(*msg);
    Joints joints = kinematics_.jsToJoints(real_state);
    feedback_joint_pos_ = joints.pos;
    feedback_joint_vel_ = joints.vel;

    FKinResult fkin = kinematics_.runForwardKinematics(feedback_joint_pos_);
    current_robot_state_.pos.x = fkin.pose(0);
    current_robot_state_.pos.y = fkin.pose(1);
    current_robot_state_.pos.z = fkin.pose(2);
    current_robot_state_.wrist_angle = fkin.pose(3);
    current_robot_state_.wrist_roll = fkin.pose(4);
    current_robot_state_.gripper = fkin.pose(5);
    robot_state_pub_.publish(current_robot_state_);
}

void ArmController::setSplinesPose(Vector6d start_pos, Vector6d start_vel, Vector6d goal_pos, ros::Time t_start) {
    double dist = (start_pos - goal_pos).head(3).norm();
    double avg_trans_speed = 0.2;
    double biggest_turn = (start_pos - goal_pos).tail(3).cwiseAbs().maxCoeff();
    double avg_rot_speed = 2.0;
    ros::Duration move_dur = ros::Duration(max(2.0, max(dist / avg_trans_speed, biggest_turn / avg_rot_speed)));
    for (int i = 0; i < 6; i++) {
        spline_managers_[i].setSpline(start_pos(i), start_vel(i), goal_pos(i), 0.0, t_start, move_dur);
    }
}

void ArmController::setSplinesJoint(Vector6d start_pos, Vector6d start_vel, Vector6d goal_pos, ros::Time t_start) {
    double biggest_turn = (start_pos - goal_pos).cwiseAbs().maxCoeff();
    double avg_rot_speed = 2.0;
    ros::Duration move_dur = ros::Duration(max(2.0, biggest_turn / avg_rot_speed));
    for (int i = 0; i < 6; i++) {
        spline_managers_[i].setSpline(start_pos(i), start_vel(i), goal_pos(i), 0.0, t_start, move_dur);
    }
}

void ArmController::setSplinesThrow(Vector6d start_pos, Vector6d start_vel, Vector6d goal_pos, Vector6d goal_vel, ros::Time t_start) {
    // Solution to prevent windup
    double time_elbow = 3*(goal_pos(ELBOW) - start_pos(ELBOW)) / (2*start_vel(ELBOW) + goal_vel(ELBOW));
    double time_wrist = 3*(goal_pos(WRIST) - start_pos(WRIST)) / (2*start_vel(WRIST) + goal_vel(WRIST));

    double actual_time = max(time_elbow, time_wrist);
    ros::Duration move_dur = ros::Duration(actual_time);
    ros::Duration elbow_delay = ros::Duration(actual_time - time_elbow);
    ros::Duration wrist_delay = ros::Duration(actual_time - time_wrist);

    ros::Duration gripper_dur = ros::Duration(0.1);

    // Splines for yaw, shoulder, twist
    spline_managers_[YAW].setSpline(start_pos(YAW), start_vel(YAW), goal_pos(YAW), goal_vel(YAW), t_start, move_dur);
    spline_managers_[SHOULDER].setSpline(start_pos(SHOULDER), start_vel(SHOULDER), goal_pos(SHOULDER), goal_vel(SHOULDER), t_start, move_dur);
    spline_managers_[TWIST].setSpline(start_pos(TWIST), start_vel(TWIST), goal_pos(TWIST), goal_vel(TWIST), t_start, move_dur);

    // Spline for gripper (instant)
    spline_managers_[GRIPPER].setSpline(start_pos(GRIPPER), start_vel(GRIPPER), goal_pos(GRIPPER), goal_vel(GRIPPER), t_start + move_dur - gripper_dur, gripper_dur);

    // Splines for elbow and wrist
    spline_managers_[ELBOW].setSpline(start_pos(ELBOW), start_vel(ELBOW), goal_pos(ELBOW), goal_vel(ELBOW), t_start + elbow_delay, ros::Duration(time_elbow));
    spline_managers_[WRIST].setSpline(start_pos(WRIST), start_vel(WRIST), goal_pos(WRIST), goal_vel(WRIST), t_start + wrist_delay, ros::Duration(time_wrist));
}

PosVelPair ArmController::getSplinePoints(ros::Time time) {
    PosVelPair pair;
    bool at_target = true;
    for (int i = 0; i < 6; i++) {
        SplinePoint point = spline_managers_[i].getPoint(time);
        pair.pos(i) = point.pos;
        pair.vel(i) = point.vel;
        at_target &= point.at_end;
    }
    current_robot_state_.is_at_target = at_target;
    return pair;
}