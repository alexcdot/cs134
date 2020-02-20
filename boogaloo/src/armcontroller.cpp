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

    // TODO uncomment when hooked up to actual hebi node

    // // Get initial position
    // sensor_msgs::JointState::ConstPtr msgptr;
    // msgptr = ros::topic::waitForMessage<sensor_msgs::JointState>(
    //     "/hebiros/robot/feedback/joint_state", nh_
    // );
    // if (msgptr == NULL) {
    //     ROS_ERROR("Failed to receive topic /hebiros/robot/feedback/joint_state");
    //     exit(1);
    // }

    // // Convert to joints and get pose using kinematics
    // Joints initial_joints = kinematics_.jsToJoints(kinematics_.fromHebi(*msgptr));
    Joints initial_joints;
    initial_joints.pos << 0,0,0,0,0,0;
    initial_joints.vel << 0.1,0,0,0,0,0;
    initial_joints.torque << 0,0,0,0,0,0;

    FKinResult initial_pose = kinematics_.runForwardKinematics(initial_joints.pos);
    
    current_joint_pos_ = initial_joints.pos;
    current_joint_vel_ = Vector6d::Zero();
    current_pose_pos_ = initial_pose.pose;
    current_pose_vel_ = Vector6d::Zero();

    current_state_ = ArmControllerState::MOVING_JOINT;
    this->setSplines(current_joint_pos_, current_joint_pos_, current_joint_vel_, ros::Time::now());

    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/tippoint", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tippose", 10);
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/hebiros/robot/command/joint_state", 10
    );
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/commanded_joints", 10
    );

    // Goal subscribers
    tip_goal_sub_ = nh_.subscribe("/joint_goal", 10, &ArmController::processJointCommand, this);
    joint_goal_sub_ = nh_.subscribe("/tip_goal", 10, &ArmController::processPoseCommand, this);

    run_timer_ = nh_.createTimer(ros::Duration(0.01), &ArmController::runController, this);
    run_timer_.start();

    ros::spin();
}

void ArmController::runController(const ros::TimerEvent& time) {
    ros::Time t = time.current_real;

    sensor_msgs::JointState joint_state;

    if (current_state_ == ArmControllerState::MOVING_POINT) {
        // Get tip pos/vel
        PosVelPair tip = this->getSplinePoints(t);

        // Get joints using ikin
        Vector6d target_joint = kinematics_.runInverseKinematics(tip.pos);

        // Get joint vel using fkin jacobian
        FKinResult fkin = kinematics_.runForwardKinematics(target_joint);
        Vector6d target_joint_vel = fkin.jacobian.inverse() * tip.vel;

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(target_joint, target_joint_vel);

        // Update current values
        current_joint_pos_ = target_joint;
        current_joint_vel_ = target_joint_vel;
        current_pose_pos_ = tip.pos;
        current_pose_vel_ = tip.vel;
        geometry_msgs::PointStamped point;
        point.header.frame_id = "world";
        point.point.x = fkin.pose(0);
        point.point.y = fkin.pose(1);
        point.point.z = fkin.pose(2);
        point_pub_.publish(point);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.pose.position.x = fkin.pose(0);
        pose.pose.position.y = fkin.pose(1);
        pose.pose.position.z = fkin.pose(2);
        pose.pose.orientation.x = fkin.orientation.x();
        pose.pose.orientation.y = fkin.orientation.y();
        pose.pose.orientation.z = fkin.orientation.z();
        pose.pose.orientation.w = fkin.orientation.w();
        pose_pub_.publish(pose);
    }
    else if (current_state_ == ArmControllerState::MOVING_JOINT) {
        // Get joint pos/vel
        PosVelPair joints = this->getSplinePoints(t);

        // Put joint targets in joint state
        joint_state = kinematics_.jointsToJS(joints.pos, joints.vel);

        // Run fkin to get jacobian for later
        FKinResult fkin = kinematics_.runForwardKinematics(joints.pos);

        // Update current values
        current_joint_pos_ = joints.pos;
        current_joint_vel_ = joints.vel;
        current_pose_pos_ = fkin.pose;
        current_pose_vel_ = fkin.jacobian * joints.vel;

        // cout << current_joint_pos_ << endl << endl;
        // cout << current_joint_vel_ << endl;
        geometry_msgs::PointStamped point;
        point.header.frame_id = "world";
        point.point.x = fkin.pose(0);
        point.point.y = fkin.pose(1);
        point.point.z = fkin.pose(2);
        point_pub_.publish(point);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.pose.position.x = fkin.pose(0);
        pose.pose.position.y = fkin.pose(1);
        pose.pose.position.z = fkin.pose(2);
        pose.pose.orientation.x = fkin.orientation.x();
        pose.pose.orientation.y = fkin.orientation.y();
        pose.pose.orientation.z = fkin.orientation.z();
        pose.pose.orientation.w = fkin.orientation.w();
        pose_pub_.publish(pose);
    }



    joint_state_pub_.publish(joint_state);
    joint_command_pub_.publish(kinematics_.toHebi(joint_state));
}

void ArmController::processJointCommand(const boogaloo::JointCommand::ConstPtr& msg) {
    ros::Time curr_time = ros::Time::now();
    Vector6d joint_command;
    joint_command << msg->yaw, msg->shoulder, msg->elbow,
                     msg->wrist, msg->twist, msg->gripper;

    if (msg->pose_follow) {
        current_state_ = ArmControllerState::MOVING_POINT;
        FKinResult fkin = kinematics_.runForwardKinematics(joint_command);
        this->setSplines(fkin.pose, current_pose_pos_, current_pose_vel_, curr_time);
    } 
    else {
        current_state_ = ArmControllerState::MOVING_JOINT;
        this->setSplines(joint_command, current_joint_pos_, current_joint_vel_, curr_time);
    }
}

void ArmController::processPoseCommand(const boogaloo::PoseCommand::ConstPtr& msg) {
    ros::Time curr_time = ros::Time::now();
    Vector6d pose_command;
    pose_command << msg->pos.x, msg->pos.y, msg->pos.z,
                    msg->wrist_angle, msg->wrist_roll, msg->gripper;

    if (msg->pose_follow) {
        current_state_ = ArmControllerState::MOVING_POINT;
        this->setSplines(pose_command, current_pose_pos_, current_pose_vel_, curr_time);
    } 
    else {
        current_state_ = ArmControllerState::MOVING_JOINT;
        Vector6d joint_command = kinematics_.runInverseKinematics(pose_command);
        this->setSplines(joint_command, current_joint_pos_, current_joint_vel_, curr_time);
    }
}


// void ArmController::processJointState(const sensor_msgs::JointState::ConstPtr& msg) {
//     Vector6d joints = Vector6d::Zero();
//     for (int i = 0; i < 6; i++) {
//         joints(i) = msg->position[i];
//     }
//     joints(5) = 0;
//     FKinResult res = kinematics_.runForwardKinematics(joints);

//     // Create point message
//     geometry_msgs::PointStamped point;
//     point.header.frame_id = "world";
//     point.point.x = res.pose(0);
//     point.point.y = res.pose(1);
//     point.point.z = res.pose(2);
//     point_pub_.publish(point);

//     // Create pose message
//     geometry_msgs::PoseStamped pose;
//     pose.header.frame_id = "world";
//     pose.pose.position.x = res.pose(0);
//     pose.pose.position.y = res.pose(1);
//     pose.pose.position.z = res.pose(2);
//     pose.pose.orientation.x = res.orientation.x();
//     pose.pose.orientation.y = res.orientation.y();
//     pose.pose.orientation.z = res.orientation.z();
//     pose.pose.orientation.w = res.orientation.w();
//     pose_pub_.publish(pose);
    
//     Vector6d guess;
//     guess << -1.57, 0, 0, 0, 0, 0;

//     Vector6d rebuilt = kinematics_.runInverseKinematics(res.pose, guess);
    
//     cout << endl << res.jacobian << endl << endl;

//     cout << "----" << endl;
// }

void ArmController::setSplines(Vector6d goal_pos, Vector6d start_pos, Vector6d start_vel, ros::Time t_start) {
    for (int i = 0; i < 6; i++) {
        spline_managers_[i].setSpline(goal_pos(i), start_pos(i), start_vel(i), t_start);
    }
}

PosVelPair ArmController::getSplinePoints(ros::Time time) {
    PosVelPair pair;
    for (int i = 0; i < 6; i++) {
        SplinePoint point = spline_managers_[i].getPoint(time);
        pair.pos(i) = point.pos;
        pair.vel(i) = point.vel;
    }
    return pair;
}