#include "armcontroller.hpp"

int main(int argc, char **argv) {
    // Initialize the basic ROS node.
    ros::init(argc, argv, "fkinnode");
    
    ArmController controller = ArmController();

    

  // Create and run a servo loop at 100Hz until shutdown.
    ros::Rate servo(100);
    double    dt = servo.expectedCycleTime().toSec();
    ROS_INFO("Running the servo loop with dt %f", dt);

    ros::Time starttime = ros::Time::now();
    ros::Time servotime;
    while(ros::ok()) {
        // Current time (since start).
        servotime = ros::Time::now();
        double t = (servotime - starttime).toSec();
        //controller.runController(t);
        // Wait for next turn.
        ros::spinOnce();
        servo.sleep();

    }

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
    this->setSplinesJoint(current_joint_pos_, current_joint_pos_, current_joint_vel_, ros::Time::now());

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
        this->setSplinesPose(fkin.pose, current_pose_pos_, current_pose_vel_, curr_time);
    } 
    else {
        current_state_ = ArmControllerState::MOVING_JOINT;
        this->setSplinesJoint(joint_command, current_joint_pos_, current_joint_vel_, curr_time);
    }
}

void ArmController::processPoseCommand(const boogaloo::PoseCommand::ConstPtr& msg) {
    ros::Time curr_time = ros::Time::now();
    Vector6d pose_command;
    pose_command << msg->pos.x, msg->pos.y, msg->pos.z,
                    msg->wrist_angle, msg->wrist_roll, msg->gripper;

    if (msg->pose_follow) {
        current_state_ = ArmControllerState::MOVING_POINT;
        this->setSplinesPose(pose_command, current_pose_pos_, current_pose_vel_, curr_time);
    } 
    else {
        current_state_ = ArmControllerState::MOVING_JOINT;
        Vector6d joint_command = kinematics_.runInverseKinematics(pose_command);
        this->setSplinesJoint(joint_command, current_joint_pos_, current_joint_vel_, curr_time);
    }
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

void ArmController::setSplinesPose(Vector6d goal_pos, Vector6d start_pos, Vector6d start_vel, ros::Time t_start) {
    double dist = (start_pos - goal_pos).head(3).norm();
    double avg_trans_speed = 0.2;
    double biggest_turn = (start_pos - goal_pos).tail(3).cwiseAbs().maxCoeff();
    double avg_rot_speed = 2.0;
    ros::Duration move_dur = ros::Duration(max(2.0, max(dist / avg_trans_speed, biggest_turn / avg_rot_speed)));
    for (int i = 0; i < 6; i++) {
        spline_managers_[i].setSpline(start_pos(i), start_vel(i), goal_pos(i), 0.0, t_start, move_dur);
    }
}

void ArmController::setSplinesJoint(Vector6d goal_pos, Vector6d start_pos, Vector6d start_vel, ros::Time t_start) {
    double biggest_turn = (start_pos - goal_pos).cwiseAbs().maxCoeff();
    double avg_rot_speed = 2.0;
    ros::Duration move_dur = ros::Duration(max(2.0, biggest_turn / avg_rot_speed));
    for (int i = 0; i < 6; i++) {
        spline_managers_[i].setSpline(start_pos(i), start_vel(i), goal_pos(i), 0.0, t_start, move_dur);
    }
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