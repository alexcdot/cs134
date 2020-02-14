#include "armcontroller.hpp"

int main(int argc, char **argv) {
    // Initialize the basic ROS node.
    ros::init(argc, argv, "fkinnode");
    
    ArmController controller = ArmController();

    sensor_msgs::JointState::ConstPtr msgptr;
    msgptr = ros::topic::waitForMessage<sensor_msgs::JointState>(
        "/hebiros/robot/feedback/joint_state", nh_
    );
    if (msgptr == NULL) {
        ROS_ERROR("Failed to receive topic /hebiros/robot/feedback/joint_state");
        return 1;
    }

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
        controller.runController(t);
        // Wait for next turn.
        ros::spinOnce();
        servo.sleep();

    }

    return 0;
}

ArmController::ArmController() {
    kinematics_ = Kinematics();

    arm_state_sub_ = nh_.subscribe("joint_states", 10, &ArmController::processJointState, this);
    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/tippoint", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tippose", 10);

    // Goal subscribers
    tip_goal_sub_ = nh_.subscribe("/joint_goal", 10, &ArmController::processJointCommand, this);
    joint_goal_sub_ = nh_.subscribe("/tip_goal", 10, &ArmController::processPoseCommand, this);

    // Command publisher
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/hebiros/robot/command/joint_state", 10
    );

    ROS_INFO("Fkin: Running...");
    ros::spin();
}

void ArmController::runController(const ros::TimerEvent& time) {
    // TODO: Fix this so it's relative to some start time
    
    double t = time.current_real();
    // TODO: Maybe use a mutex here
    sensor_msgs::JointState command_msg;
    command_msg.position.resize(6);
    command_msg.velocity.resize(6);
    command_msg.effort.resize(6);

    if (current_state_ == ArmControllerState::STOPPED) {
        return;
    }
    else if (current_state_ == ArmControllerState::MOVING_POINT) {
        Vector6d target_tip;
        Vector6d target_tip_velocity;

        for (int i = 0; i < 6; i++) {
            SplinePoint q = spline_managers_[i].getPoint(t);
            target_tip[i] = q.pos;
            target_tip_velocity[i] = q.vel;
        }

        // TODO: Improve the guess
        Vector6d guess;
        guess << -1.57, 0, 0, 0, 0, 0;
        Vector6d target_joint = kinematics_.runInverseKinematics(target_tip, guess);

        FKinResult fkin = kinematics_.runForwardKinematics(target_joint);
        Vector6d target_joint_velocity = fkin.jacobian.inverse() * target_tip_velocity;

        for (int i = 0; i < 6; i++) {
            command_msg.name.push_back(kinematics_.JointNames[i]);
            command_msg.position[i] = target_joint[i];
            command_msg.velocity[i] = target_joint_velocity[i];
        }
    }
    else if (current_state_ == ArmControllerState::MOVING_JOINT) {
        for (int i = 0; i < 6; i++) {
            SplinePoint q = spline_managers_[i].getPoint(t);
            command_msg.name.push_back(kinematics_.JointNames[i]);
            command_msg.position[i] = q.pos;
            command_msg.velocity[i] = q.vel;
        }
    }
    joint_command_pub_.publish(command_msg);
}

void ArmController::processJointCommand(const boogaloo::JointCommand::ConstPtr& msg) {
    // TODO: switch to moving joint mode
    current_state_ = ArmControllerState::MOVING_JOINT;
}

void ArmController::processPoseCommand(const boogaloo::PoseCommand::ConstPtr& msg) {
    // TODO: switch to moving tip mode
    current_state_ = ArmControllerState::MOVING_POINT;
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
    
    cout << endl << res.jacobian << endl << endl;

    cout << "----" << endl;
}

